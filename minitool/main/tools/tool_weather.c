/*
 * tool_weather.c — Clima actual usando IP-API y Open-Meteo.
 * Tarea HTTP asíncrona para no bloquear la interfaz gráfica.
 */
#include "tool.h"
#include "wifi_manager.h"
#include "bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "weather";

/* Declaramos la fuente generada */
LV_FONT_DECLARE(font_weather_28);
LV_FONT_DECLARE(font_text_16);

/* Buffers de interfaz */
static lv_obj_t *s_city_label = NULL;
static lv_obj_t *s_temp_label = NULL;
static lv_obj_t *s_emoji_label = NULL;
static lv_obj_t *s_desc_label = NULL;
static lv_obj_t *s_refresh_btn = NULL;
static volatile bool s_fetching = false;

/* Estructura para separar emoji de texto */
typedef struct {
    const char* emoji;
    const char* text;
} weather_info_t;

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
        case 95: case 96: case 99: return (weather_info_t){"⛈️", "Tormenta eléctrica"};
        default: return (weather_info_t){"☁️", "..."}; 
    }
}

/* 
 * Función robusta para hacer HTTP GET y leer la respuesta completa en bucle,
 * incluso si el router nos manda los datos fragmentados en varios paquetes.
 */
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
        /* Leer en bucle hasta juntar todos los pedazos del JSON */
        while (total_read < max_len - 1) {
            int read_len = esp_http_client_read(client, buffer + total_read, max_len - 1 - total_read);
            if (read_len <= 0) break; /* Ya no hay más datos */
            total_read += read_len;
        }
        buffer[total_read] = '\0';
        success = (total_read > 0);
    }
    esp_http_client_cleanup(client);
    return success;
}

/* 
 * Tarea de FreeRTOS para descarga y parseo
 */
static void fetch_weather_task(void *pvParameters) {
    /* Variables locales que mantienen el último estado válido */
    static char last_city[32] = "Ubicando...";
    char local_temp[16] = "--.- °""C";
    char local_emoji[16] = "☁️";
    char local_desc[32] = "Error de red";
    float lat = 0.0f, lon = 0.0f;

    /* Pedir memoria RAM (2KB) de forma segura para los JSONs grandes */
    char *json_buffer = malloc(2048); 
    
    if (json_buffer != NULL) {
        /* 1. OBTENER UBICACIÓN POR IP */
        if (fetch_http_to_buffer("http://ip-api.com/json/?fields=lat,lon,city", json_buffer, 2048)) {
            cJSON *json = cJSON_Parse(json_buffer);
            if (json) {
                cJSON *city = cJSON_GetObjectItem(json, "city");
                cJSON *jlat = cJSON_GetObjectItem(json, "lat");
                cJSON *jlon = cJSON_GetObjectItem(json, "lon");
                if (city && jlat && jlon) {
                    snprintf(last_city, sizeof(last_city), "%s", city->valuestring);
                    lat = jlat->valuedouble;
                    lon = jlon->valuedouble;
                }
                cJSON_Delete(json);
            }
        }

        /* 2. OBTENER CLIMA SI TENEMOS COORDENADAS */
        if (lat != 0.0f && lon != 0.0f) {
            char weather_url[180];
            snprintf(weather_url, sizeof(weather_url), 
                     "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current_weather=true", 
                     lat, lon);
                     
            if (fetch_http_to_buffer(weather_url, json_buffer, 2048)) {
                cJSON *json = cJSON_Parse(json_buffer);
                if (json) {
                    cJSON *current = cJSON_GetObjectItem(json, "current_weather");
                    if (current) {
                        cJSON *temp = cJSON_GetObjectItem(current, "temperature");
                        cJSON *wcode = cJSON_GetObjectItem(current, "weathercode");
                        if (temp && wcode) {
                            snprintf(local_temp, sizeof(local_temp), "%.1f \xC2\xB0""C", temp->valuedouble);
                            
                            /* Separar código en Emoji y Texto */
                            weather_info_t wi = get_weather_info(wcode->valueint);
                            snprintf(local_emoji, sizeof(local_emoji), "%s", wi.emoji);
                            snprintf(local_desc, sizeof(local_desc), "%s", wi.text);
                        }
                    }
                    cJSON_Delete(json);
                }
            }
        }
        /* Liberar la memoria RAM una vez terminado el trabajo */
        free(json_buffer);
    }

    /* 3. ACTUALIZAR LA PANTALLA (Bajo el Lock de LVGL) */
    if (bsp_lvgl_lock(200)) {
        if (s_city_label) lv_label_set_text(s_city_label, last_city);
        if (s_temp_label) lv_label_set_text(s_temp_label, local_temp);
        if (s_emoji_label) lv_label_set_text(s_emoji_label, local_emoji);
        if (s_desc_label) lv_label_set_text(s_desc_label, local_desc);
        bsp_lvgl_unlock();
    }

    s_fetching = false;
    vTaskDelete(NULL);
}

/* Evento del botón */
static void refresh_click_cb(lv_event_t *e) {
    (void)e;
    if (!wifi_manager_is_connected()) {
        lv_label_set_text(s_city_label, "Sin Wi-Fi");
        return;
    }
    
    if (!s_fetching) {
        s_fetching = true;
        lv_label_set_text(s_city_label, "Ubicando...");
        lv_label_set_text(s_temp_label, "--.- \xC2\xB0""C");
        if (s_emoji_label) lv_label_set_text(s_emoji_label, "☁️");
        if (s_desc_label) lv_label_set_text(s_desc_label, "Descargando...");
        
        xTaskCreate(fetch_weather_task, "weather_fetch", 4096, NULL, 5, NULL);
    }
}

static void weather_open(lv_obj_t *parent) {
    s_fetching = false;

    /* Ciudad (Arriba) */
    s_city_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_city_label, &font_text_16, 0); 
    lv_obj_set_style_text_color(s_city_label, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_city_label, "Esperando red...");
    lv_obj_align(s_city_label, LV_ALIGN_TOP_MID, 0, 30);

    

    /* Temperatura (Centro) */
    s_temp_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_temp_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_temp_label, lv_color_white(), 0);
    lv_label_set_text(s_temp_label, "--.- \xC2\xB0""C");
    lv_obj_center(s_temp_label);

    /* Contenedor flexible para alinear emoji y texto horizontalmente */
    lv_obj_t *desc_cont = lv_obj_create(parent);
    lv_obj_remove_style_all(desc_cont);
    lv_obj_set_size(desc_cont, 200, 40);
    lv_obj_align(desc_cont, LV_ALIGN_CENTER, 0, 45);
    lv_obj_set_flex_flow(desc_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(desc_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(desc_cont, 8, 0); /* 8px de separación entre emoji y texto */

    /* Emoji (Grande, 28px) */
    s_emoji_label = lv_label_create(desc_cont);
    lv_obj_set_style_text_font(s_emoji_label, &font_weather_28, 0);
    lv_obj_set_style_text_color(s_emoji_label, lv_color_white(), 0); /* <-- ESTA ES LA LÍNEA NUEVA */
    lv_label_set_text(s_emoji_label, "☁️");

    /* Texto (Pequeño, 16px) */
    s_desc_label = lv_label_create(desc_cont);
    lv_obj_set_style_text_font(s_desc_label, &font_text_16, 0);
    lv_obj_set_style_text_color(s_desc_label, lv_color_hex(0x35D07F), 0);
    lv_label_set_text(s_desc_label, "Toca refrescar");

    /* Botón */
    s_refresh_btn = lv_btn_create(parent);
    lv_obj_set_size(s_refresh_btn, 130, 36);
    lv_obj_align(s_refresh_btn, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_radius(s_refresh_btn, 18, 0);
    lv_obj_set_style_bg_color(s_refresh_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_refresh_btn, refresh_click_cb, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *btn_lbl = lv_label_create(s_refresh_btn);
    lv_label_set_text(btn_lbl, LV_SYMBOL_REFRESH " Refrescar");
    lv_obj_center(btn_lbl);

    if (wifi_manager_is_connected()) {
        refresh_click_cb(NULL);
    }
}

static void weather_close(void) {
    /* La tarea de FreeRTOS revisará estos punteros antes de actualizar */
    s_city_label = NULL;
    s_temp_label = NULL;
    s_emoji_label = NULL;
    s_desc_label = NULL;
    s_refresh_btn = NULL;
}

const tool_t tool_weather = {
    .name = "Clima",
    .icon = LV_SYMBOL_IMAGE, 
    .accent = 0xF1C40F, 
    .open = weather_open,
    .close = weather_close
};