/*
 * weather_service.c — Descarga y caché del clima (ubicación IP + Open-Meteo).
 *
 * Extraído desde tool_weather para poder compartir el dato entre la tool y el
 * screensaver. La descarga corre en una tarea de FreeRTOS y el resultado se
 * guarda bajo mutex; los consumidores leen con weather_service_get().
 */
#include "weather_service.h"
#include "wifi_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_log.h"
#include "nvs.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

static const char *TAG = "weather_svc";

#define MIN_REFRESH_SECONDS 600  /* no re-descargar más seguido que esto */

/* Espera tras un intento fallido. Arranca corta y se va duplicando hasta el
 * intervalo normal: sin esto, un fallo dejaba al servicio reintentando cada
 * vez que el watchface refrescaba (~5 s), para siempre. Además de gastar red
 * y batería, esa tormenta es la mejor forma de que ip-api.com — que limita por
 * IP — te bloquee y el fallo se vuelva permanente. */
#define RETRY_FIRST_SECONDS  60

#define GEO_NS   "weather"
#define GEO_LAT  "lat"
#define GEO_LON  "lon"
#define GEO_CITY "city"

static SemaphoreHandle_t s_mutex = NULL;
static weather_data_t    s_cache;
static uint32_t          s_generation = 0;
static volatile bool     s_fetching = false;
static time_t            s_last_update = 0;   /* última descarga BUENA */
static time_t            s_last_try = 0;      /* último intento, salga o no */
static int               s_retry_s = 0;       /* espera actual tras fallar */

typedef struct { const char *emoji; const char *text; } weather_info_t;

static weather_info_t get_weather_info(int code) {
    switch (code) {
        case 0: return (weather_info_t){"☀️", "Despejado"};
        case 1: case 2: return (weather_info_t){"⛅", "Parcial"};
        case 3: return (weather_info_t){"☁️", "Nublado"};
        case 45: case 48: return (weather_info_t){"🌫️", "Niebla"};
        case 51: case 53: case 55: return (weather_info_t){"🌧️", "Llovizna"};
        case 56: case 57: return (weather_info_t){"🌧️", "Llovizna helada"};
        case 61: case 63: case 65: return (weather_info_t){"🌧️", "Lluvia"};
        case 66: case 67: return (weather_info_t){"🌧️", "Lluvia helada"};
        case 71: case 73: case 75: case 77: return (weather_info_t){"❄️", "Nieve"};
        case 80: case 81: case 82: return (weather_info_t){"🌧️", "Chubascos"};
        case 85: case 86: return (weather_info_t){"❄️", "Tormentas nieve"};
        case 95: case 96: case 99: return (weather_info_t){"⛈️", "Tormenta"};
        default: return (weather_info_t){"☁️", "..."};
    }
}

static bool fetch_http_to_buffer(const char *url, char *buffer, size_t max_len) {
    memset(buffer, 0, max_len);
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 4000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return false;

    bool success = false;
    if (esp_http_client_open(client, 0) == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int total_read = 0;
        while (total_read < (int)max_len - 1) {
            int read_len = esp_http_client_read(client, buffer + total_read, max_len - 1 - total_read);
            if (read_len <= 0) break;
            total_read += read_len;
        }
        buffer[total_read] = '\0';
        success = (total_read > 0);
    }
    esp_http_client_cleanup(client);
    return success;
}

static void publish(const char *city, const char *temp, const char *emoji, const char *desc) {
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (city)  strlcpy(s_cache.city,  city,  sizeof(s_cache.city));
    if (temp)  strlcpy(s_cache.temp,  temp,  sizeof(s_cache.temp));
    if (emoji) strlcpy(s_cache.emoji, emoji, sizeof(s_cache.emoji));
    if (desc)  strlcpy(s_cache.desc,  desc,  sizeof(s_cache.desc));
    s_cache.valid = true;
    s_generation++;
    xSemaphoreGive(s_mutex);
}

/* La ubicación se guarda en NVS: cambia poquísimo, y así una caída de
 * ip-api.com (o su límite por IP) no deja al reloj sin clima. */
static void geo_load(float *lat, float *lon, char *city, size_t city_size) {
    nvs_handle_t h;
    if (nvs_open(GEO_NS, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, GEO_LAT, &v) == ESP_OK) *lat = (float)v / 10000.0f;
    if (nvs_get_i32(h, GEO_LON, &v) == ESP_OK) *lon = (float)v / 10000.0f;
    size_t sz = city_size;
    nvs_get_str(h, GEO_CITY, city, &sz);
    nvs_close(h);
}

static void geo_save(float lat, float lon, const char *city) {
    nvs_handle_t h;
    if (nvs_open(GEO_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, GEO_LAT, (int32_t)(lat * 10000.0f));
    nvs_set_i32(h, GEO_LON, (int32_t)(lon * 10000.0f));
    nvs_set_str(h, GEO_CITY, city);
    nvs_commit(h);
    nvs_close(h);
}

static void fetch_weather_task(void *pv) {
    (void)pv;
    char city[32] = "Ubicando...";
    char temp[16] = "--.- \xC2\xB0""C";
    char emoji[16] = "☁️";
    char desc[32] = "Error de red";
    float lat = 0.0f, lon = 0.0f;
    bool ok = false;

    /* Si ya sabemos dónde estamos, se saltea la geolocalización. */
    geo_load(&lat, &lon, city, sizeof(city));

    char *json = malloc(2048);
    if (json != NULL) {
        if ((lat == 0.0f && lon == 0.0f) &&
            fetch_http_to_buffer("http://ip-api.com/json/?fields=lat,lon,city", json, 2048)) {
            cJSON *root = cJSON_Parse(json);
            if (root) {
                cJSON *jc = cJSON_GetObjectItem(root, "city");
                cJSON *jlat = cJSON_GetObjectItem(root, "lat");
                cJSON *jlon = cJSON_GetObjectItem(root, "lon");
                if (jc && jlat && jlon) {
                    snprintf(city, sizeof(city), "%s", jc->valuestring);
                    lat = jlat->valuedouble;
                    lon = jlon->valuedouble;
                    geo_save(lat, lon, city);   /* no volver a pedirla */
                }
                cJSON_Delete(root);
            }
        }

        if (lat != 0.0f && lon != 0.0f) {
            char url[180];
            snprintf(url, sizeof(url),
                     "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current_weather=true",
                     lat, lon);
            if (fetch_http_to_buffer(url, json, 2048)) {
                cJSON *root = cJSON_Parse(json);
                if (root) {
                    cJSON *cur = cJSON_GetObjectItem(root, "current_weather");
                    if (cur) {
                        cJSON *jt = cJSON_GetObjectItem(cur, "temperature");
                        cJSON *jw = cJSON_GetObjectItem(cur, "weathercode");
                        if (jt && jw) {
                            snprintf(temp, sizeof(temp), "%.1f \xC2\xB0""C", jt->valuedouble);
                            weather_info_t wi = get_weather_info(jw->valueint);
                            snprintf(emoji, sizeof(emoji), "%s", wi.emoji);
                            snprintf(desc, sizeof(desc), "%s", wi.text);
                            ok = true;
                        }
                    }
                    cJSON_Delete(root);
                }
            }
        }
        free(json);
    }

    publish(city, temp, emoji, desc);   /* si falló, al menos deja la ciudad */

    if (ok) {
        time(&s_last_update);
        s_retry_s = 0;
    } else {
        /* Backoff: 60 s, 2 min, 4 min... hasta el intervalo normal. */
        s_retry_s = (s_retry_s == 0) ? RETRY_FIRST_SECONDS : s_retry_s * 2;
        if (s_retry_s > MIN_REFRESH_SECONDS) s_retry_s = MIN_REFRESH_SECONDS;
        ESP_LOGW(TAG, "Descarga fallida; próximo intento en %d s", s_retry_s);
    }

    s_fetching = false;
    vTaskDelete(NULL);
}

void weather_service_init(void) {
    if (s_mutex) return;
    s_mutex = xSemaphoreCreateMutex();
    memset(&s_cache, 0, sizeof(s_cache));
    snprintf(s_cache.emoji, sizeof(s_cache.emoji), "☁️");
    snprintf(s_cache.temp, sizeof(s_cache.temp), "--.- \xC2\xB0""C");
}

void weather_service_refresh(bool force) {
    if (!s_mutex) weather_service_init();
    if (s_fetching) return;
    if (!wifi_manager_is_connected()) return;

    /* El límite se aplica sobre el último INTENTO, no sobre la última descarga
     * buena: si se mira s_last_update, un fallo lo deja en 0 y el servicio
     * reintenta en cada refresco del watchface, o sea cada pocos segundos. */
    time_t now; time(&now);
    if (!force && s_last_try != 0) {
        int wait = (s_retry_s > 0) ? s_retry_s : MIN_REFRESH_SECONDS;
        if (now - s_last_try < wait) return;
    }
    s_last_try = now;

    s_fetching = true;
    if (xTaskCreate(fetch_weather_task, "weather_fetch", 4096, NULL, 5, NULL) != pdPASS) {
        s_fetching = false;
        ESP_LOGW(TAG, "No se pudo crear la tarea de descarga");
    }
}

bool weather_service_get(weather_data_t *out) {
    if (!out) return false;
    if (!s_mutex) { out->valid = false; return false; }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_cache;
    xSemaphoreGive(s_mutex);
    return out->valid;
}

uint32_t weather_service_generation(void) { return s_generation; }
bool weather_service_is_fetching(void) { return s_fetching; }
