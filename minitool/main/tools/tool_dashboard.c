/*
 * tool_dashboard.c — Vista de monitoreo IoT.
 *
 * El MQTT y la lógica de alertas viven ahora en alert_service (segundo plano),
 * de modo que las alertas flotantes aparecen en cualquier pantalla. Esta tool
 * es solo la vista detallada: refresca su UI leyendo el estado del servicio.
 *
 * Muestra dos cosas de un vistazo:
 *   - la puerta del refri (arriba, en grande),
 *   - la humedad del suelo de la planta (abajo), con el color del estado de su
 *     umbral, para no tener que entrar a la tool Sensores solo para ver si hay
 *     que regar.
 *
 * A futuro, cuando el dashboard reciba alertas de más equipos, basta con
 * añadir fuentes en alert_service y ampliar esta vista.
 */
#include "tool.h"
#include "alert_service.h"
#include "sensor_service.h"
#include "sensor_alert.h"

#include "esp_log.h"

#include <stdio.h>
#include <string.h>

/* Buffers de interfaz */
static lv_obj_t *s_title_label = NULL;
static lv_obj_t *s_icon_label = NULL;
static lv_obj_t *s_status_label = NULL;
static lv_obj_t *s_plant_label = NULL;   /* fila inferior: la planta */
static lv_timer_t *s_refresh_timer = NULL;

/* Primer sensor de suelo que haya (normalmente el de la maceta). -1 si no hay. */
static int find_soil_sensor(void) {
    int n = sensor_count();
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX];
        if (!sensor_get(i, id, sizeof(id), NULL, 0, NULL)) continue;
        if (strcmp(sensor_leaf(id), "suelo") == 0) return i;
    }
    return -1;
}

static void render_plant(void) {
    if (!s_plant_label) return;

    int idx = find_soil_sensor();
    if (idx < 0) {                       /* todavía no llegó ninguna lectura */
        lv_label_set_text(s_plant_label, "");
        return;
    }

    char id[SENSOR_ID_MAX], val[16];
    uint32_t age = 0;
    sensor_get(idx, id, sizeof(id), val, sizeof(val), &age);

    bool stale = (age > sensor_alert_stale_limit(id));
    sensor_alert_state_t st = sensor_alert_state(id);

    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_TINT "  %s %%%s", val, stale ? "  (viejo)" : "");
    lv_label_set_text(s_plant_label, buf);

    lv_color_t c = lv_color_hex(0x35D07F);                 /* normal: verde */
    if (stale)                              c = lv_color_hex(0x7F8C8D);
    else if (st == SENSOR_ALERT_LOW)        c = lv_color_hex(0xE74C3C);  /* hay que regar */
    else if (st == SENSOR_ALERT_HIGH)       c = lv_color_hex(0xE0A030);  /* encharcada */
    lv_obj_set_style_text_color(s_plant_label, c, 0);
}

static void render(void) {
    render_plant();
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
        lv_obj_set_style_text_color(s_icon_label, lv_color_hex(0x35D07F), 0);
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

    s_plant_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_plant_label, &lv_font_montserrat_16, 0);
    lv_label_set_text(s_plant_label, "");
    lv_obj_align(s_plant_label, LV_ALIGN_BOTTOM_MID, 0, -34);

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
    s_plant_label = NULL;
}

const tool_t tool_dashboard = {
    .name = "Dashboard",
    .icon = LV_SYMBOL_LIST,
    .accent = 0x3498DB,
    .open = dashboard_open,
    .close = dashboard_close
};
