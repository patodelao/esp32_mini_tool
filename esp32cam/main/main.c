/*
 * main.c — Nodo 'cam' (AI-Thinker ESP32-CAM).
 *
 * Red + telemetría + cámara, todo actualizable por Wi-Fi. El primer flasheo por
 * cable ya está hecho; de acá en más el binario entra por OTA (POST /update),
 * igual que en el refri.
 *
 * La foto se ve en http://<ip>/foto.jpg desde cualquier navegador de la red.
 * La cámara arranca al boot pero NO es crítica: si falla (placa sin sensor,
 * PSRAM que no monta), el nodo sigue vivo como nodo OTA y /foto.jpg responde
 * 503, en vez de un cuelgue que obligaría a volver al cable.
 *
 * El nodo se integra al home-lab con las mismas convenciones que los demás:
 *   labo/nodo/cam/status   online/offline (retenido, con last-will)
 *   labo/nodo/cam/ip       IP para llegarle
 *   labo/nodo/cam/cmd      comandos (reset, leer)
 *   labo/sensor/cam/rssi   dBm    (retenido)
 *   labo/sensor/cam/uptime minutos
 *   labo/sensor/cam/heap   kB libres
 *   labo/alerta/cam        JSON multicanal
 * Así aparece solo en las tools Nodos y Sensores del minitool, sin tocar nada.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

#include "ota_web.h"
#include "camera.h"

/* Credenciales fuera del fuente: van en secrets.h (.gitignore). Copiá
 * secrets.h.example a secrets.h y completá el tuyo. Si falta, compila igual
 * pero con relleno que no conecta a nada. */
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

/* LED rojo de a bordo del AI-Thinker ESP32-CAM: GPIO33, activo en bajo. Es la
 * única señal de vida sin serial, que en esta placa es lo habitual. Parpadea
 * mientras busca red; queda fijo cuando MQTT está conectado. (NO se usa el
 * GPIO4, que es el flash blanco y encandila.) */
#define LED_GPIO        GPIO_NUM_33
#define LED_ON          0
#define LED_OFF         1

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1

/* Broker propio en la Raspberry Pi (Mosquitto), IP fija por reserva DHCP.
 * Antes: broker.hivemq.com (público). */
#define MQTT_BROKER_URI    "mqtt://192.168.1.100"
#define TELEMETRY_MS       60000

#define DEVICE_ID          "cam"
#define MQTT_TOPIC_STATUS  "labo/nodo/"   DEVICE_ID "/status"
#define MQTT_TOPIC_IP      "labo/nodo/"   DEVICE_ID "/ip"
#define MQTT_TOPIC_CMD     "labo/nodo/"   DEVICE_ID "/cmd"
#define MQTT_TOPIC_ALERT   "labo/alerta/" DEVICE_ID
#define MQTT_TOPIC_RSSI    "labo/sensor/" DEVICE_ID "/rssi"
#define MQTT_TOPIC_UPTIME  "labo/sensor/" DEVICE_ID "/uptime"
#define MQTT_TOPIC_HEAP    "labo/sensor/" DEVICE_ID "/heap"

static const char *TAG = "cam";
static EventGroupHandle_t s_net_events;
static esp_mqtt_client_handle_t s_mqtt;

static bool mqtt_ready(void)
{
    if (!s_mqtt || !s_net_events) return false;
    EventBits_t b = xEventGroupGetBits(s_net_events);
    return (b & (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT)) ==
           (WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
}

/* --------------------------------- Publicar ------------------------------- */

static void publish_ip(void)
{
    if (!mqtt_ready()) return;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    if (netif && esp_netif_get_ip_info(netif, &info) == ESP_OK && info.ip.addr) {
        char buf[24];
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&info.ip));
        esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_IP, buf, 0, 1, 1);
    }
}

/* Alerta estructurada al bus multicanal (la consumen minitool, app, etc.). */
static void publish_alert(const char *nivel, const char *msg)
{
    if (!mqtt_ready()) return;
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"origen\":\"Camara\",\"nivel\":\"%s\",\"msg\":\"%s\"}", nivel, msg);
    esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_ALERT, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Alerta [%s] %s", nivel, msg);
}

/* Telemetría de salud como sensores numéricos retenidos: la tool Sensores los
 * grafica sola. El "online" se reafirma en cada ronda —no solo al conectar—
 * porque tras un reinicio el last-will retenido del broker puede llegar
 * después del "online" nuevo y pisarlo; insistir lo repara solo. */
static void publish_salud(void)
{
    if (!mqtt_ready()) return;
    char buf[24];

    esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_STATUS, "online", 0, 1, 1);

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%d", ap.rssi);
        esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_RSSI, buf, 0, 1, 1);
    }
    snprintf(buf, sizeof(buf), "%llu", esp_timer_get_time() / 60000000ULL);
    esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_UPTIME, buf, 0, 1, 1);
    snprintf(buf, sizeof(buf), "%.1f", esp_get_free_heap_size() / 1024.0f);
    esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_HEAP, buf, 0, 1, 1);
}

/* ------------------------------- Eventos ---------------------------------- */

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args; (void)base;

    if (id == MQTT_EVENT_CONNECTED) {
        xEventGroupSetBits(s_net_events, MQTT_CONNECTED_BIT);
        ESP_LOGI(TAG, "MQTT conectado");
        gpio_set_level(LED_GPIO, LED_ON);   /* señal de vida: fijo = conectado */
        publish_salud();
        publish_ip();
        esp_mqtt_client_subscribe(s_mqtt, MQTT_TOPIC_CMD, 1);
        /* El servidor de OTA se levanta recién con red arriba. */
        ota_web_start(OTA_PASSWORD, publish_alert);
    } else if (id == MQTT_EVENT_DATA) {
        esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
        char cmd[24];
        int n = ev->data_len < (int)sizeof(cmd) - 1 ? ev->data_len : (int)sizeof(cmd) - 1;
        memcpy(cmd, ev->data, n);
        cmd[n] = '\0';

        ESP_LOGI(TAG, "CMD: %s", cmd);
        if (strcmp(cmd, "reset") == 0 || strcmp(cmd, "reiniciar") == 0) {
            publish_alert("ok", "Reiniciando el nodo");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        } else if (strcmp(cmd, "leer") == 0) {
            publish_salud();
            publish_ip();
        }
    } else if (id == MQTT_EVENT_DISCONNECTED) {
        xEventGroupClearBits(s_net_events, MQTT_CONNECTED_BIT);
        ESP_LOGW(TAG, "MQTT desconectado");
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_net_events, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
        gpio_set_level(LED_GPIO, LED_OFF);
        ESP_LOGW(TAG, "Wi-Fi caido, reintentando");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_net_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi conectado");
    }
}

/* ------------------------------- Arranque --------------------------------- */

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable = true;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mqtt_start(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
        .network.disable_auto_reconnect = false,
        /* Testamento (LWT): si el nodo cae de golpe, el broker publica
           "offline" por él. */
        .session.last_will = {
            .topic = MQTT_TOPIC_STATUS,
            .msg = "offline",
            .msg_len = 0,
            .qos = 1,
            .retain = 1,
        },
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
}

/* Latido del LED mientras no hay MQTT: da una señal de vida en una placa sin
 * consola. Cuando conecta, el handler lo deja fijo y esta tarea no lo pisa. */
static void led_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!mqtt_ready()) {
            gpio_set_level(LED_GPIO, LED_ON);
            vTaskDelay(pdMS_TO_TICKS(120));
            gpio_set_level(LED_GPIO, LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(880));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Nodo cam (ESP32-CAM)");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, LED_OFF);

    /* La cámara arranca antes que la red, pero su fallo NO frena nada: el nodo
       tiene que quedar accesible por OTA aunque el sensor no responda. */
    if (camera_init() != ESP_OK) {
        ESP_LOGW(TAG, "Camara no disponible; el nodo sigue como nodo OTA");
    }

    s_net_events = xEventGroupCreate();
    xTaskCreate(led_task, "led", 2048, NULL, 2, NULL);

    wifi_start();
    mqtt_start();

    /* Telemetría periódica. La reconexión de Wi-Fi/MQTT es automática, así que
       este bucle solo publica; si la red está caída, mqtt_ready() lo omite. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_MS));
        publish_salud();
    }
}
