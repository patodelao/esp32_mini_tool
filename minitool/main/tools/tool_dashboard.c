/*
 * tool_dashboard.c — Monitor IoT MQTT
 * Se suscribe a HiveMQ y usa el heartbeat de 5s como Watchdog para tolerancia a fallos.
 */
#include "tool.h"
#include "wifi_manager.h"
#include "bsp.h"

#include "mqtt_client.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "dashboard";

/* Buffers de interfaz y hardware */
static lv_obj_t *s_title_label = NULL;
static lv_obj_t *s_icon_label = NULL;
static lv_obj_t *s_status_label = NULL;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

/* El temporizador que vigilará el heartbeat de 5 segundos */
static lv_timer_t *s_watchdog_timer = NULL;

/* Variables persistentes (sobreviven cuando cierras y abres la tool) */
static bool s_door_is_open = false;
static bool s_has_data = false;

/* Actualiza la pantalla en base a la variable persistente */
static void update_dashboard_ui(void) {
    if (!s_icon_label || !s_status_label) return;
    
    if (s_door_is_open) {
        lv_label_set_text(s_icon_label, LV_SYMBOL_WARNING);
        lv_obj_set_style_text_color(s_icon_label, lv_color_hex(0xE74C3C), 0);
        lv_label_set_text(s_status_label, "¡PUERTA ABIERTA!");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xE74C3C), 0);
    } else {
        lv_label_set_text(s_icon_label, LV_SYMBOL_OK);
        lv_obj_set_style_text_color(s_icon_label, lv_color_hex(0x2ECC71), 0);
        lv_label_set_text(s_status_label, "Puerta Cerrada");
        lv_obj_set_style_text_color(s_status_label, lv_color_white(), 0);
    }
}

/* Evento del Watchdog: Si pasan 12 segundos sin escuchar al refri, cerramos la puerta por software */
static void heartbeat_timeout_cb(lv_timer_t *timer) {
    ESP_LOGW(TAG, "Watchdog expiró: Sin heartbeat por 12s. Forzando estado CERRADO.");
    s_door_is_open = false;
    lv_timer_pause(timer); /* Detenemos el watchdog hasta que la vuelvan a abrir */
    update_dashboard_ui();
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Conectado. Suscribiendo...");
            esp_mqtt_client_subscribe(s_mqtt_client, "proyectos/casa/refri/puerta", 1);
            break;

        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, "proyectos/casa/refri/puerta", event->topic_len) == 0) {
                s_has_data = true;
                bool is_open_msg = (strncmp(event->data, "ABIERTO", event->data_len) == 0);
                
                if (is_open_msg) {
                    s_door_is_open = true;
                    /* ¡Llegó el heartbeat! Reseteamos los 12 segundos del watchdog */
                    if (s_watchdog_timer) {
                        lv_timer_reset(s_watchdog_timer);
                        lv_timer_resume(s_watchdog_timer);
                    }
                } else {
                    s_door_is_open = false;
                    /* La puerta se cerró legalmente, pausamos el watchdog */
                    if (s_watchdog_timer) lv_timer_pause(s_watchdog_timer);
                }
                
                if (bsp_lvgl_lock(200)) {
                    update_dashboard_ui();
                    bsp_lvgl_unlock();
                }
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            if (bsp_lvgl_lock(200)) {
                if (s_status_label && !s_has_data) {
                    lv_label_set_text(s_status_label, "Reconectando...");
                }
                bsp_lvgl_unlock();
            }
            break;
            
        default:
            break;
    }
}

static void dashboard_open(lv_obj_t *parent) {
    s_title_label = lv_label_create(parent);
    lv_obj_set_style_text_color(s_title_label, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_title_label, "Sensor Refri");
    lv_obj_align(s_title_label, LV_ALIGN_TOP_MID, 0, 20);

    s_icon_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_icon_label, &lv_font_montserrat_48, 0);
    lv_label_set_text(s_icon_label, LV_SYMBOL_MINUS);
    lv_obj_align(s_icon_label, LV_ALIGN_CENTER, 0, -10);

    s_status_label = lv_label_create(parent);
    lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, 40);

    /* Creamos el Watchdog en 12 segundos (pausado por defecto para no ejecutarlo sin querer) */
    s_watchdog_timer = lv_timer_create(heartbeat_timeout_cb, 12000, NULL);
    lv_timer_pause(s_watchdog_timer);

    if (!wifi_manager_is_connected()) {
        lv_label_set_text(s_status_label, "Sin conexión Wi-Fi");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x7F8C8D), 0);
    } else {
        if (s_has_data) {
            update_dashboard_ui();
            /* Si la memoria decía que la puerta quedó abierta la última vez, 
               arrancamos el watchdog para verificar si todavía siguen llegando heartbeats */
            if (s_door_is_open) {
                lv_timer_resume(s_watchdog_timer);
            }
        } else {
            lv_label_set_text(s_status_label, "Conectando al Broker...");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x7F8C8D), 0);
        }
        
        esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = "mqtt://broker.hivemq.com",
        };
        s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(s_mqtt_client);
    }
}

static void dashboard_close(void) {
    /* Es vital borrar el timer de LVGL antes de destruir la pantalla */
    if (s_watchdog_timer) {
        lv_timer_del(s_watchdog_timer);
        s_watchdog_timer = NULL;
    }
    
    if (s_mqtt_client) {
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }
    
    s_title_label = NULL;
    s_icon_label = NULL;
    s_status_label = NULL;
}

const tool_t tool_dashboard = {
    .name = "Dashboard",
    .icon = LV_SYMBOL_LIST, 
    .accent = 0x3498DB, 
    .open = dashboard_open,
    .close = dashboard_close
};