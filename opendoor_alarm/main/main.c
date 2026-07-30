#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#include "ota_web.h"
#include "mqtt_broker.h"

/* Credenciales fuera del fuente: van en secrets.h, que está en .gitignore.
 * Copiá secrets.h.example a secrets.h y completá el tuyo. Si falta, el
 * proyecto compila igual pero con valores de relleno que no conectan a nada. */
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "cambiame"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "cambiame"
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "cambiame"
#endif
/* Credenciales del broker. NULL = anónimo (broker sin auth). Para broker con
 * password, definí MQTT_USER/MQTT_PASS en secrets.h. */
#ifndef MQTT_USER
#define MQTT_USER NULL
#endif
#ifndef MQTT_PASS
#define MQTT_PASS NULL
#endif

#define DOOR_SENSOR_GPIO GPIO_NUM_10
#define DOOR_OPEN_LEVEL 1

#define BUZZER_GPIO GPIO_NUM_5

#define WS2812_GPIO GPIO_NUM_48
#define WS2812_RMT_CHANNEL RMT_CHANNEL_0
#define WS2812_T0H 14
#define WS2812_T0L 34
#define WS2812_T1H 28
#define WS2812_T1L 24

#define RGB_RED_GPIO GPIO_NUM_2
#define RGB_GREEN_GPIO GPIO_NUM_3
#define RGB_BLUE_GPIO GPIO_NUM_4

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define LED_START_DELAY_MS 20000
#define BUZZER_START_DELAY_MS 30000
#define ALARM_BLINK_MS 250
#define DOOR_OPEN_HEARTBEAT_MS 5000
#define DOOR_CLOSED_CONFIRM_MS 3000

/* Este nodo ya no se conecta a un broker externo: HOSPEDA el broker del home-lab
 * (ver mqtt_broker.c). Publica su telemetría/puerta en su propio broker por la
 * API in-process, y el resto de la flota se conecta a él por TCP en el 1883. */

/* --- Topics ---------------------------------------------------------------
 * Se mantiene el topic original de la puerta (compatibilidad con el dashboard
 * y la alerta ya existentes en el minitool) y se agregan los topics según las
 * convenciones del minitool para que este equipo aparezca solo en las tools
 * Nodos (estado) y Sensores (RSSI/duración), y en el bus de alertas multicanal.
 */
#define DEVICE_ID           "refri"
/* Estado de la puerta. Va bajo labo/nodo/<id>/ y no bajo labo/sensor/<id>/
 * porque el payload es texto (ABIERTO/CERRADO), no un numero: los sensores del
 * minitool asumen numeros para graficar y aplicar umbrales. */
#define MQTT_TOPIC_DOOR     "labo/nodo/" DEVICE_ID "/puerta"  /* ABIERTO/CERRADO */
#define MQTT_TOPIC_STATUS   "labo/nodo/" DEVICE_ID "/status" /* online/offline   */
#define MQTT_TOPIC_ALERT    "labo/alerta/" DEVICE_ID         /* JSON multicanal  */
#define MQTT_TOPIC_RSSI     "labo/sensor/" DEVICE_ID "/rssi"   /* dBm            */
#define MQTT_TOPIC_UPTIME   "labo/sensor/" DEVICE_ID "/uptime" /* minutos        */
#define MQTT_TOPIC_HEAP     "labo/sensor/" DEVICE_ID "/heap"   /* kB libres      */
#define MQTT_TOPIC_IP       "labo/nodo/"   DEVICE_ID "/ip"     /* para llegarle  */
#define MQTT_TOPIC_OPENSECS "labo/sensor/" DEVICE_ID "/abierta_seg"
#define MQTT_TOPIC_CMD      "labo/nodo/"   DEVICE_ID "/cmd"   /* tool Control */

static const char *TAG = "door_alarm";
static EventGroupHandle_t s_net_events;
static bool s_wifi_shutdown_requested;
static bool s_ws2812_ready;
static bool s_alarm_escalated;

static bool door_is_open(void)
{
    return gpio_get_level(DOOR_SENSOR_GPIO) == DOOR_OPEN_LEVEL;
}

static void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ws2812_ready) {
        return;
    }

    rmt_item32_t items[25];
    uint8_t grb[3] = { g, r, b };
    int item_idx = 0;

    for (int byte_idx = 0; byte_idx < 3; byte_idx++) {
        uint8_t value = grb[byte_idx];
        for (int bit = 7; bit >= 0; bit--) {
            bool one = ((value >> bit) & 0x01) != 0;
            items[item_idx].level0 = 1;
            items[item_idx].duration0 = one ? WS2812_T1H : WS2812_T0H;
            items[item_idx].level1 = 0;
            items[item_idx].duration1 = one ? WS2812_T1L : WS2812_T0L;
            item_idx++;
        }
    }

    items[item_idx].level0 = 0;
    items[item_idx].duration0 = 2000;
    items[item_idx].level1 = 0;
    items[item_idx].duration1 = 0;

    ESP_ERROR_CHECK(rmt_write_items(WS2812_RMT_CHANNEL, items, item_idx + 1, true));
    ESP_ERROR_CHECK(rmt_wait_tx_done(WS2812_RMT_CHANNEL, pdMS_TO_TICKS(20)));
}

static void door_sensor_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << DOOR_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static void buzzer_init(void)
{
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2500,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const ledc_channel_config_t channel_cfg = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void buzzer_set(bool enabled)
{
    uint32_t duty = enabled ? 512 : 0;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void rgb_led_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << RGB_RED_GPIO) | (1ULL << RGB_GREEN_GPIO) | (1ULL << RGB_BLUE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    const rmt_config_t rmt_tx_cfg = {
        .rmt_mode = RMT_MODE_TX,
        .channel = WS2812_RMT_CHANNEL,
        .gpio_num = WS2812_GPIO,
        .clk_div = 2,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        },
    };

    esp_err_t err = rmt_config(&rmt_tx_cfg);
    if (err == ESP_OK) {
        err = rmt_driver_install(WS2812_RMT_CHANNEL, 0, 0);
    }

    if (err == ESP_OK) {
        s_ws2812_ready = true;
        ESP_LOGI(TAG, "WS2812 habilitado en GPIO %d", WS2812_GPIO);
    } else {
        s_ws2812_ready = false;
        ESP_LOGW(TAG, "WS2812 no disponible (%s), usando solo RGB por pines", esp_err_to_name(err));
    }
}

static void rgb_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_set_color(r, g, b);
    ESP_ERROR_CHECK(gpio_set_level(RGB_RED_GPIO, r > 0 ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(RGB_GREEN_GPIO, g > 0 ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(RGB_BLUE_GPIO, b > 0 ? 1 : 0));
}

static void alarm_outputs_off(void)
{
    buzzer_set(false);
    rgb_led_set(0, 0, 0);
}

static void led_startup_test(void)
{
    rgb_led_set(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    rgb_led_set(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    rgb_led_set(0, 0, 0);
}

/* "Listo para publicar" = el broker local está escuchando. El transporte ahora
 * es in-process (este nodo ES el broker), así que ya no dependemos de un cliente
 * conectado a nadie: mientras el broker corra, publicar siempre funciona y sus
 * retenidos quedan guardados para cuando un cliente (el minitool) se suscriba. */
static bool mqtt_ready(void)
{
    return mqtt_broker_running();
}

/* Estado del nodo para la tool Nodos (retenido: el último valor persiste). */
static void publish_status(const char *state)
{
    if (!mqtt_ready()) {
        return;
    }
    mqtt_broker_local_publish(MQTT_TOPIC_STATUS, state, true /*retain*/);
    ESP_LOGI(TAG, "Estado nodo -> %s", state);
}

/* Alerta estructurada al bus multicanal (la consumen minitool, app, Node-RED…). */
static void publish_alert(const char *nivel, const char *msg)
{
    if (!mqtt_ready()) {
        return;
    }
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"origen\":\"Refri\",\"nivel\":\"%s\",\"msg\":\"%s\"}", nivel, msg);
    mqtt_broker_local_publish(MQTT_TOPIC_ALERT, payload, false);
    ESP_LOGI(TAG, "Alerta [%s] %s", nivel, msg);
}

/* Telemetría de salud como sensores numéricos (retenidos): la tool Sensores
 * los grafica y les aplica umbrales sola, igual que a los del nodo pieza. */
static void publish_salud(void)
{
    if (!mqtt_ready()) {
        return;
    }
    char buf[24];

    /* El "online" va en cada ronda de telemetria, no solo al conectar. Si el
       nodo se reinicia (un OTA, por ejemplo), el broker tarda unos segundos en
       notar que la conexion vieja murio y recien ahi dispara su last-will
       retenido; para entonces la sesion nueva ya publico su "online", asi que
       el "offline" llega despues y pisa el valor bueno. Insistir cada minuto
       lo repara solo, y son 13 bytes. */
    mqtt_broker_local_publish(MQTT_TOPIC_STATUS, "online", true /*retain*/);

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%d", ap.rssi);
        mqtt_broker_local_publish(MQTT_TOPIC_RSSI, buf, true /*retain*/);
    }

    snprintf(buf, sizeof(buf), "%llu", esp_timer_get_time() / 60000000ULL);
    mqtt_broker_local_publish(MQTT_TOPIC_UPTIME, buf, true /*retain*/);

    snprintf(buf, sizeof(buf), "%.1f", esp_get_free_heap_size() / 1024.0f);
    mqtt_broker_local_publish(MQTT_TOPIC_HEAP, buf, true /*retain*/);
}

/* IP con la que se llega al nodo; la muestra la tool Nodos. */
static void publish_ip(void)
{
    if (!mqtt_ready()) {
        return;
    }
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    if (netif && esp_netif_get_ip_info(netif, &info) == ESP_OK && info.ip.addr) {
        char buf[24];
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&info.ip));
        mqtt_broker_local_publish(MQTT_TOPIC_IP, buf, true /*retain*/);
    }
}

/* Comandos del refri desde la tool Control (labo/nodo/refri/cmd). Antes llegaban
 * por el evento DATA del cliente MQTT; ahora, como este nodo ES el broker, se
 * reciben por la API in-process (mqtt_broker_on_local). Corre en la task del
 * broker: mantenerlo liviano. cmd llega terminado en '\0'. */
static void on_cmd(const char *topic, const char *cmd, int len)
{
    (void)topic; (void)len;
    ESP_LOGI(TAG, "CMD: %s", cmd);
    if (strcmp(cmd, "reset") == 0 || strcmp(cmd, "reiniciar") == 0) {
        publish_alert("ok", "Reiniciando el nodo");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (strcmp(cmd, "leer") == 0) {
        publish_salud();
        publish_ip();
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error iniciando Wi-Fi: %s", esp_err_to_name(err));
        }
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_net_events, WIFI_CONNECTED_BIT);
        if (!s_wifi_shutdown_requested) {
            ESP_LOGW(TAG, "Wi-Fi desconectado, reintentando");
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error reconectando Wi-Fi: %s", esp_err_to_name(err));
            }
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_net_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi conectado");
        /* Publicar la IP apenas la tenemos (y en cada reconexión / cambio de
           IP), sin esperar la ronda de telemetría. Si el broker aún no arrancó,
           el guard de publish_ip lo omite y la telemetría lo cubre luego. */
        publish_ip();
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_wait_connected(void)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_net_events,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        ESP_LOGW(TAG, "No se obtuvo conexión Wi-Fi en el tiempo esperado");
    }
}

static esp_err_t publish_door_state(const char *payload)
{
    if (!mqtt_ready()) {
        ESP_LOGW(TAG, "Broker no listo, no se publica %s", payload);
        return ESP_FAIL;
    }
    mqtt_broker_local_publish(MQTT_TOPIC_DOOR, payload, false);
    ESP_LOGI(TAG, "Puerta publicada [%s]", payload);
    return ESP_OK;
}

static void publish_door_open_heartbeat(void)
{
    if (!mqtt_ready()) {
        ESP_LOGW(TAG, "Heartbeat ABIERTO omitido: broker no listo");
        return;
    }
    mqtt_broker_local_publish(MQTT_TOPIC_DOOR, "ABIERTO", false);
    ESP_LOGI(TAG, "Heartbeat ABIERTO enviado");
}

static bool confirm_door_closed_for_ms(uint32_t confirm_ms)
{
    uint32_t elapsed_ms = 0;
    const uint32_t check_step_ms = 100;

    while (elapsed_ms < confirm_ms) {
        if (door_is_open()) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(check_step_ms));
        elapsed_ms += check_step_ms;
    }

    return !door_is_open();
}

static void alarm_task(void *arg)
{
    (void)arg;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_net_events = xEventGroupCreate();
    if (s_net_events == NULL) {
        ESP_LOGE(TAG, "No se pudo crear EventGroup");
        vTaskDelete(NULL);
        return;
    }

    door_sensor_init();
    buzzer_init();
    rgb_led_init();
    alarm_outputs_off();
    led_startup_test();

    wifi_start();
    wifi_wait_connected();

    /* Este nodo ES el broker del home-lab: en vez de conectarse a un broker
       ajeno, levanta el suyo (Mongoose, escuchando en el 1883) y publica su
       telemetria/puerta ahi mismo por la API in-process. El cmd se registra
       ANTES de arrancar la task del broker para no competir por su tabla. */
    mqtt_broker_on_local(MQTT_TOPIC_CMD, on_cmd);
    mqtt_broker_start();

    /* Anunciar presencia y arrancar el OTA ahora (antes lo disparaba el evento
       de conexion del cliente MQTT, que ya no existe). */
    publish_status("online");
    publish_salud();
    publish_ip();
    ota_web_start(OTA_PASSWORD, publish_alert);

    /* Vigilancia continua. Este nodo esta enchufado, asi que no duerme: se
       queda mirando la puerta, manda telemetria periodica y deja el servidor
       de OTA escuchando todo el tiempo. Antes se dormia apenas terminaba y
       despertaba por GPIO, lo que ahorraba bateria que no hace falta ahorrar y
       hacia imposible actualizarlo o pedirle nada. */
    bool was_open = false;
    uint32_t open_time_ms = 0;
    uint32_t heartbeat_ms = 0;
    uint32_t telemetry_ms = 0;
    bool red_phase = true;

    while (true) {
        bool open = door_is_open();

        if (open && !was_open) {
            was_open = true;
            open_time_ms = 0;
            heartbeat_ms = 0;
            red_phase = true;
            s_alarm_escalated = false;
            publish_door_state("ABIERTO");
            publish_alert("aviso", "Puerta abierta");
            ESP_LOGW(TAG, "Puerta ABIERTA: LED a %d ms, buzzer a %d ms",
                     LED_START_DELAY_MS, BUZZER_START_DELAY_MS);
        } else if (!open && was_open) {
            /* Callar PRIMERO, confirmar después. El iman rebota y por eso el
               cierre se confirma durante unos segundos, pero antes eso pasaba
               con el buzzer todavía sonando: cerrabas la puerta y seguía
               pitando. La confirmación es silenciosa; si resulta que la puerta
               se reabrió, la vuelta siguiente del bucle vuelve a encender todo
               sola (was_open sigue en true y open_time_ms ya pasó los
               umbrales), así que no se pierde nada por apagar antes de tiempo. */
            alarm_outputs_off();

            if (confirm_door_closed_for_ms(DOOR_CLOSED_CONFIRM_MS)) {
                was_open = false;
                publish_door_state("CERRADO");
                publish_alert("ok", "Puerta cerrada");
                if (mqtt_ready()) {
                    char buf[16];
                    snprintf(buf, sizeof(buf), "%u", (unsigned)(open_time_ms / 1000));
                    mqtt_broker_local_publish(MQTT_TOPIC_OPENSECS, buf, true);
                }
                ESP_LOGI(TAG, "Puerta CERRADA confirmada");
            } else {
                ESP_LOGW(TAG, "Reabierta durante la confirmacion");
            }
        }

        if (was_open) {
            bool led_active = open_time_ms >= LED_START_DELAY_MS;
            bool buzzer_active = open_time_ms >= BUZZER_START_DELAY_MS;

            if (led_active) rgb_led_set(red_phase ? 255 : 0, red_phase ? 0 : 255, 0);
            else            rgb_led_set(0, 0, 0);
            buzzer_set(buzzer_active && red_phase);

            if (buzzer_active && !s_alarm_escalated) {
                s_alarm_escalated = true;
                publish_alert("alarma", "Puerta abierta demasiado tiempo");
            }

            red_phase = !red_phase;
            open_time_ms += ALARM_BLINK_MS;
            heartbeat_ms += ALARM_BLINK_MS;
            if (heartbeat_ms >= DOOR_OPEN_HEARTBEAT_MS) {
                heartbeat_ms = 0;
                publish_door_open_heartbeat();
            }
        }

        /* Telemetria de salud al mismo ritmo que los demas nodos. Sin esto sus
           sensores quedarian viejos para siempre y el minitool avisaria que el
           refri esta sin datos. */
        telemetry_ms += ALARM_BLINK_MS;
        if (telemetry_ms >= 60000) {
            telemetry_ms = 0;
            publish_salud();
            /* Republicar la IP acá y no solo al arrancar: al boot todavía no hay
               IP (info.ip.addr == 0) y no se publicaba nunca; además re-siembra
               el retenido tras un reinicio del broker (vive en RAM). Así el
               refri aparece con su IP en la tool Nodos, igual que los demás. */
            publish_ip();
        }

        vTaskDelay(pdMS_TO_TICKS(ALARM_BLINK_MS));
    }
}

void app_main(void)
{
    xTaskCreate(alarm_task, "alarm_task", 6144, NULL, 5, NULL);
}
