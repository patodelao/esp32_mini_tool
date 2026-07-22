#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/rtc_io.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#define WIFI_SSID "DonPatoysusecuaces"
#define WIFI_PASSWORD "Armando1910"

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
#define MQTT_CONNECTED_BIT BIT1
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define NETWORK_CONNECT_TIMEOUT_MS 15000
#define LED_START_DELAY_MS 20000
#define BUZZER_START_DELAY_MS 30000
#define ALARM_BLINK_MS 250
#define DOOR_OPEN_HEARTBEAT_MS 5000
#define DOOR_CLOSED_CONFIRM_MS 3000

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com"

/* --- Topics ---------------------------------------------------------------
 * Se mantiene el topic original de la puerta (compatibilidad con el dashboard
 * y la alerta ya existentes en el minitool) y se agregan los topics según las
 * convenciones del minitool para que este equipo aparezca solo en las tools
 * Nodos (estado) y Sensores (RSSI/duración), y en el bus de alertas multicanal.
 */
#define DEVICE_ID           "refri"
#define MQTT_TOPIC_DOOR     "proyectos/casa/refri/puerta"   /* ABIERTO/CERRADO  */
#define MQTT_TOPIC_STATUS   "labo/nodo/" DEVICE_ID "/status" /* online/offline   */
#define MQTT_TOPIC_ALERT    "labo/alerta/" DEVICE_ID         /* JSON multicanal  */
#define MQTT_TOPIC_RSSI     "labo/sensor/" DEVICE_ID "/rssi" /* dBm              */
#define MQTT_TOPIC_OPENSECS "labo/sensor/" DEVICE_ID "/abierta_seg"

static const char *TAG = "door_alarm";
static EventGroupHandle_t s_net_events;
static bool s_wifi_shutdown_requested;
static bool s_ws2812_ready;
static bool s_alarm_escalated;
static esp_mqtt_client_handle_t s_mqtt_client;

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

static bool is_cold_boot(void)
{
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED;
}

static bool mqtt_ready(void)
{
    if (s_mqtt_client == NULL || s_net_events == NULL) {
        return false;
    }
    EventBits_t bits = xEventGroupGetBits(s_net_events);
    return (bits & (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT)) == (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
}

/* Estado del nodo para la tool Nodos (retenido: el último valor persiste). */
static void publish_status(const char *state)
{
    if (!mqtt_ready()) {
        return;
    }
    esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_STATUS, state, 0, 1, 1 /*retain*/);
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
    esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_ALERT, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Alerta [%s] %s", nivel, msg);
}

/* Señal Wi-Fi como sensor numérico (retenido) para graficar en la tool Sensores. */
static void publish_rssi(void)
{
    if (!mqtt_ready()) {
        return;
    }
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", ap.rssi);
        esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_RSSI, buf, 0, 1, 1 /*retain*/);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    if (event_id == MQTT_EVENT_CONNECTED) {
        xEventGroupSetBits(s_net_events, MQTT_CONNECTED_BIT);
        ESP_LOGI(TAG, "MQTT conectado");
        /* Anunciar presencia y señal en cuanto hay conexión */
        esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_STATUS, "online", 0, 1, 1 /*retain*/);
        publish_rssi();
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        xEventGroupClearBits(s_net_events, MQTT_CONNECTED_BIT);
        ESP_LOGW(TAG, "MQTT desconectado");
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
        xEventGroupClearBits(s_net_events, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
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

static void mqtt_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .network.disable_auto_reconnect = false,
        /* Testamento (LWT): si el nodo cae de golpe (corte de energía, pérdida
           de Wi-Fi sin cierre limpio), el broker publica "offline" por él. */
        .session.last_will = {
            .topic = MQTT_TOPIC_STATUS,
            .msg = "offline",
            .msg_len = 0,
            .qos = 1,
            .retain = 1,
        },
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));
}

static esp_err_t wait_for_network_ready(void)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_net_events,
        WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(NETWORK_CONNECT_TIMEOUT_MS));

    if ((bits & (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT)) != (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT)) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t publish_door_state(const char *payload)
{
    esp_err_t net_err = wait_for_network_ready();
    if (net_err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo publicar %s: red no lista", payload);
        return net_err;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_DOOR, payload, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Falló publish de %s", payload);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MQTT publicado [%s] msg_id=%d", payload, msg_id);
    return ESP_OK;
}

static void publish_door_open_heartbeat(void)
{
    EventBits_t bits = xEventGroupGetBits(s_net_events);
    if ((bits & (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT)) != (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Heartbeat ABIERTO omitido: MQTT no conectado");
        return;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_DOOR, "ABIERTO", 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Falló heartbeat ABIERTO");
        return;
    }

    ESP_LOGI(TAG, "Heartbeat ABIERTO enviado msg_id=%d", msg_id);
}

static void run_alarm_until_closed(uint32_t *open_time_ms, uint32_t *heartbeat_elapsed_ms, bool *red_phase)
{
    while (door_is_open()) {
        bool led_active = *open_time_ms >= LED_START_DELAY_MS;
        bool buzzer_active = *open_time_ms >= BUZZER_START_DELAY_MS;

        if (led_active) {
            if (*red_phase) {
                rgb_led_set(255, 0, 0);
            } else {
                rgb_led_set(0, 255, 0);
            }
        } else {
            rgb_led_set(0, 0, 0);
        }

        buzzer_set(buzzer_active && *red_phase);

        /* Escalada a "alarma" (multicanal) la primera vez que suena el buzzer */
        if (buzzer_active && !s_alarm_escalated) {
            s_alarm_escalated = true;
            publish_alert("alarma", "Puerta abierta demasiado tiempo");
        }

        vTaskDelay(pdMS_TO_TICKS(ALARM_BLINK_MS));

        *open_time_ms += ALARM_BLINK_MS;
        *heartbeat_elapsed_ms += ALARM_BLINK_MS;
        if (*heartbeat_elapsed_ms >= DOOR_OPEN_HEARTBEAT_MS) {
            publish_door_open_heartbeat();
            publish_rssi();
            *heartbeat_elapsed_ms = 0;
        }
        *red_phase = !*red_phase;
    }
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

static void configure_deep_sleep_wakeup(void)
{
    if (!rtc_gpio_is_valid_gpio(DOOR_SENSOR_GPIO)) {
        ESP_LOGE(TAG, "GPIO %d no soporta wakeup RTC", DOOR_SENSOR_GPIO);
        return;
    }

    ESP_ERROR_CHECK(rtc_gpio_pullup_en(DOOR_SENSOR_GPIO));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(DOOR_SENSOR_GPIO));

#if SOC_PM_SUPPORT_EXT0_WAKEUP
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(DOOR_SENSOR_GPIO, DOOR_OPEN_LEVEL));
    ESP_LOGI(TAG, "Wakeup EXT0 configurado");
#elif SOC_PM_SUPPORT_EXT1_WAKEUP
    esp_sleep_ext1_wakeup_mode_t mode;
#ifdef ESP_EXT1_WAKEUP_ANY_LOW
    mode = DOOR_OPEN_LEVEL ? ESP_EXT1_WAKEUP_ANY_HIGH : ESP_EXT1_WAKEUP_ANY_LOW;
#else
    mode = DOOR_OPEN_LEVEL ? ESP_EXT1_WAKEUP_ANY_HIGH : ESP_EXT1_WAKEUP_ALL_LOW;
#endif
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(1ULL << DOOR_SENSOR_GPIO, mode));
    ESP_LOGI(TAG, "Wakeup EXT1 configurado");
#else
    ESP_LOGW(TAG, "Target sin soporte EXT0/EXT1");
#endif
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
    if (is_cold_boot()) {
        led_startup_test();
    }

    wifi_start();
    wifi_wait_connected();
    mqtt_start();
    if (wait_for_network_ready() != ESP_OK) {
        ESP_LOGW(TAG, "Conectividad MQTT no lista al inicio, continuará con reconexión automática");
    }

    if (door_is_open()) {
        uint32_t open_time_ms = 0;
        uint32_t heartbeat_elapsed_ms = 0;
        bool red_phase = true;

        s_alarm_escalated = false;
        if (publish_door_state("ABIERTO") != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo publicar ABIERTO");
        }
        publish_alert("aviso", "Puerta abierta");
        ESP_LOGW(TAG, "Puerta ABIERTA: LED inicia a %d ms, buzzer inicia a %d ms", LED_START_DELAY_MS, BUZZER_START_DELAY_MS);

        while (true) {
            run_alarm_until_closed(&open_time_ms, &heartbeat_elapsed_ms, &red_phase);
            alarm_outputs_off();
            ESP_LOGI(TAG, "Puerta cerrada detectada, confirmando cierre por %d ms", DOOR_CLOSED_CONFIRM_MS);

            if (confirm_door_closed_for_ms(DOOR_CLOSED_CONFIRM_MS)) {
                ESP_LOGI(TAG, "Puerta CERRADA confirmada");
                if (publish_door_state("CERRADO") != ESP_OK) {
                    ESP_LOGW(TAG, "No se pudo publicar CERRADO");
                }
                publish_alert("ok", "Puerta cerrada");
                /* Duración de la apertura como sensor (segundos) */
                if (mqtt_ready()) {
                    char buf[16];
                    snprintf(buf, sizeof(buf), "%u", (unsigned)(open_time_ms / 1000));
                    esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_OPENSECS, buf, 0, 1, 1);
                }
                break;
            }

            ESP_LOGW(TAG, "Puerta reabierta durante confirmación de cierre, reactivando alarma");
        }
    }

    alarm_outputs_off();

    /* Anunciar que el nodo se va a dormir (offline limpio, retenido). Así la
       tool Nodos muestra "offline"/durmiendo en vez de esperar al watchdog. */
    publish_status("offline");
    vTaskDelay(pdMS_TO_TICKS(150)); /* dar tiempo a que salga el paquete */

    s_wifi_shutdown_requested = true;
    if (s_mqtt_client != NULL) {
        ESP_ERROR_CHECK(esp_mqtt_client_stop(s_mqtt_client));
        ESP_ERROR_CHECK(esp_mqtt_client_destroy(s_mqtt_client));
        s_mqtt_client = NULL;
    }
    ESP_ERROR_CHECK(esp_wifi_stop());

    configure_deep_sleep_wakeup();
    ESP_LOGI(TAG, "Entrando en deep sleep");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep_start();
}

void app_main(void)
{
    xTaskCreate(alarm_task, "alarm_task", 6144, NULL, 5, NULL);
}
