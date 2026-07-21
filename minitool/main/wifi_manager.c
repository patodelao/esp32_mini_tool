/*
 * wifi_manager.c — Wi-Fi STA no bloqueante + SNTP (zona horaria de Chile).
 * Credenciales persistidas en NVS (namespace "cfg", claves "ssid"/"pass").
 */
#include "wifi_manager.h"

#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sntp.h"

static const char *TAG = "wifi";

/* Valores por defecto si NVS está vacío (primer arranque) */
#define WIFI_SSID_DEFAULT "invitado"
#define WIFI_PASS_DEFAULT "111111111111111"

#define NVS_NAMESPACE "cfg"

static char s_ssid[WIFI_SSID_MAX + 1] = WIFI_SSID_DEFAULT;
static char s_pass[WIFI_PASS_MAX + 1] = WIFI_PASS_DEFAULT;

static bool s_connected = false;      /* hay IP asignada */
static bool s_should_connect = false; /* intención del usuario: mantener conexión */
static bool s_sntp_started = false;

bool wifi_manager_is_connected(void) { return s_connected; }

void wifi_manager_get_ssid(char *buf, size_t len)
{
    if (buf && len) {
        strlcpy(buf, s_ssid, len);
    }
}


static void load_credentials(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    
    size_t len = sizeof(s_ssid);
    nvs_get_str(nvs, "ssid", s_ssid, &len);
    len = sizeof(s_pass);
    nvs_get_str(nvs, "pass", s_pass, &len);
    
    /* Leemos si la última vez lo dejamos conectado (1) o desconectado (0) */
    uint8_t auto_conn = 0;
    if (nvs_get_u8(nvs, "autoconnect", &auto_conn) == ESP_OK) {
        s_should_connect = (auto_conn == 1);
    }
    
    nvs_close(nvs);
}

static void save_credentials(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo abrir NVS para guardar credenciales");
        return;
    }
    nvs_set_str(nvs, "ssid", s_ssid);
    nvs_set_str(nvs, "pass", s_pass);
    
    /* Guardamos la intención actual del usuario */
    nvs_set_u8(nvs, "autoconnect", s_should_connect ? 1 : 0);
    
    nvs_commit(nvs);
    nvs_close(nvs);
}

static void apply_credentials(void)
{
    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, s_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, s_pass, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
}

void wifi_manager_set_credentials(const char *ssid, const char *pass)
{
    if (ssid && ssid[0]) strlcpy(s_ssid, ssid, sizeof(s_ssid));
    if (pass) strlcpy(s_pass, pass, sizeof(s_pass));
    save_credentials();
    apply_credentials();
    ESP_LOGI(TAG, "Credenciales actualizadas (SSID: %s)", s_ssid);

    /* Si el usuario quería estar conectado, reconectar con lo nuevo */
    if (s_should_connect) {
        esp_wifi_disconnect(); /* el evento DISCONNECTED reintenta solo */
    }
}

static void start_sntp_once(void)
{
    if (s_sntp_started) return;
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    /* Zona horaria de Chile continental */
    setenv("TZ", "CLT4CLST,M9.1.6/23,M4.1.6/23", 1);
    tzset();
    s_sntp_started = true;
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_should_connect) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        /* Solo reintentar si el usuario quiere seguir conectado. */
        if (s_should_connect) {
            ESP_LOGI(TAG, "Desconectado, reintentando...");
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        start_sntp_once();
    }
}

void wifi_manager_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_credentials();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    apply_credentials();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_start());
    
    /* Si en la última sesión el usuario lo dejó conectado, reconectar automáticamente */
    if (s_should_connect) {
        ESP_LOGI(TAG, "Auto-conectando a red guardada: %s", s_ssid);
        esp_wifi_connect();
    }
}



void wifi_manager_connect(void)
{
    s_should_connect = true;
    save_credentials(); /* Guardar en memoria estática que queremos auto-conectar */
    ESP_LOGI(TAG, "Conectando a '%s'...", s_ssid);
    esp_wifi_connect();
}

void wifi_manager_disconnect(void)
{
    s_should_connect = false;
    save_credentials(); /* Guardar en memoria estática que NO queremos conectar */
    ESP_LOGI(TAG, "Desconectando...");
    esp_wifi_disconnect();
    s_connected = false;
}
