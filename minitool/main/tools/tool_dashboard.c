/*
 * tool_dashboard.c — Vista de monitoreo IoT.
 *
 * El MQTT y la lógica de alertas viven ahora en alert_service (segundo plano),
 * de modo que las alertas flotantes aparecen en cualquier pantalla. Esta tool
 * es solo la vista detallada: refresca su UI leyendo el estado del servicio.
 *
 * A futuro, cuando el dashboard reciba alertas de más equipos, basta con
 * añadir fuentes en alert_service y ampliar esta vista.
 */
#include "tool.h"
#include "alert_service.h"

#include "esp_log.h"

/* Buffers de interfaz */
static lv_obj_t *s_title_label = NULL;
static lv_obj_t *s_icon_label = NULL;
static lv_obj_t *s_status_label = NULL;
static lv_timer_t *s_refresh_timer = NULL;

static void render(void) {
    if (!s_icon_label || !s_status_label) return;

    if (!alert_service_mqtt_connected() && !alert_service_refri_has_data()) {
        lv_label_set_text(s_icon_label, LV_SYMBOL_WARNING);
        lv_obj_set_style_text_color(s_icon_label, lv_color_hex(0x7F8C8D), 0);
        lv_label_set_text(s_status_label, "Conectando al broker...");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x7F8C8D), 0);
        return;
    }

    if (alert_service_refri_open()) {
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

static void refresh_cb(lv_timer_t *t) {
    (void)t;
    render();
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

    render();
    s_refresh_timer = lv_timer_create(refresh_cb, 500, NULL);
}

static void dashboard_close(void) {
    if (s_refresh_timer) {
        lv_timer_del(s_refresh_timer);
        s_refresh_timer = NULL;
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
