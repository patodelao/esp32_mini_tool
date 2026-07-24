/*
 * ui_watchface.c — Carátula de reposo: hora grande + arco de segundos.
 *
 * Abajo, dos datos que uno quiere mirar de reojo sin entrar a ninguna tool:
 * los pasos del día y la humedad del suelo de la planta. Este último se pone
 * rojo cuando hay que regar, que es la única información del home-lab que
 * exige una acción concreta.
 */
#include "ui_watchface.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "lvgl.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "weather_service.h"
#include "pedometer_service.h"
#include "sensor_service.h"
#include "sensor_alert.h"
#include "ui_quick.h"

LV_FONT_DECLARE(font_weather_28);

static lv_obj_t *s_wf = NULL;
static lv_obj_t *s_arc = NULL;
static lv_obj_t *s_time_label = NULL;
static lv_obj_t *s_date_label = NULL;
static lv_obj_t *s_wifi_icon = NULL;
static lv_obj_t *s_bt_icon = NULL;
static lv_obj_t *s_wx_emoji = NULL;
static lv_obj_t *s_wx_temp = NULL;
static lv_obj_t *s_steps_lbl = NULL;   /* pasos de hoy */
static lv_obj_t *s_soil_lbl = NULL;    /* humedad del suelo de la planta */
static lv_timer_t *s_timer = NULL;
static uint32_t s_wx_gen = 0xFFFFFFFF;

bool ui_watchface_active(void) { return s_wf != NULL; }

static void wf_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    if (s_wf) {
        lv_obj_del(s_wf);
        s_wf = NULL;
    }
    s_arc = NULL;
    s_time_label = s_date_label = NULL;
    s_wifi_icon = s_bt_icon = NULL;
    s_wx_emoji = s_wx_temp = NULL;
    s_steps_lbl = s_soil_lbl = NULL;
}

/* Pasos del día, compactos: "1234" o "12.3k" para que no desborde. */
static void render_steps(void)
{
    if (!s_steps_lbl) return;
    uint32_t s = pedometer_steps();
    char buf[16];
    if (s < 10000) snprintf(buf, sizeof(buf), "%lu", (unsigned long)s);
    else           snprintf(buf, sizeof(buf), "%.1fk", (double)s / 1000.0);
    lv_label_set_text(s_steps_lbl, buf);
}

/* Humedad del suelo del primer sensor de suelo que haya. Rojo = hay que regar,
 * gris = el dato está viejo. Si no hay sensor, no se muestra nada. */
static void render_soil(void)
{
    if (!s_soil_lbl) return;

    int n = sensor_count();
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;
        if (strcmp(sensor_leaf(id), "suelo") != 0) continue;

        char buf[24];
        snprintf(buf, sizeof(buf), LV_SYMBOL_TINT " %s%%", val);
        lv_label_set_text(s_soil_lbl, buf);

        sensor_alert_state_t st = sensor_alert_state(id);
        lv_color_t c = lv_color_hex(0x8FA8C8);
        if (age > sensor_alert_stale_limit(id))   c = lv_color_hex(0x4A5560);
        else if (st == SENSOR_ALERT_LOW)          c = lv_color_hex(0xE74C3C);
        else if (st == SENSOR_ALERT_HIGH)         c = lv_color_hex(0xE0A030);
        lv_obj_set_style_text_color(s_soil_lbl, c, 0);

        lv_obj_clear_flag(s_soil_lbl, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_add_flag(s_soil_lbl, LV_OBJ_FLAG_HIDDEN);
}

/* Un gesto tambien produce un CLICKED al soltar, y la caratula se cierra con
 * cualquier toque: sin esta marca, deslizar hacia abajo abriria el panel y
 * acto seguido cerraria la caratula debajo. */
static bool s_gesture_used = false;

static void wf_click_cb(lv_event_t *e)
{
    (void)e;
    if (s_gesture_used) { s_gesture_used = false; return; }
    wf_close();
}

/* Deslizar hacia abajo abre el panel rapido, como en cualquier reloj o
 * telefono actual. */
static void wf_gesture_cb(lv_event_t *e)
{
    (void)e;
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    if (lv_indev_get_gesture_dir(indev) == LV_DIR_BOTTOM) {
        s_gesture_used = true;
        ui_quick_show();
    }
}

static void wf_tick_cb(lv_timer_t *t)
{
    (void)t;
    time_t now;
    struct tm tm_info;
    time(&now);
    localtime_r(&now, &tm_info);

    char buf[16];
    strftime(buf, sizeof(buf), "%H:%M", &tm_info);
    lv_label_set_text(s_time_label, buf);

    if (tm_info.tm_year >= (2020 - 1900)) {
        strftime(buf, sizeof(buf), "%d/%m/%Y", &tm_info);
        lv_label_set_text(s_date_label, buf);
    } else {
        lv_label_set_text(s_date_label, "--/--/----");
    }

    lv_arc_set_value(s_arc, tm_info.tm_sec);

    /* Presencia = conectado: el icono aparece solo cuando hay conexión */
    if (wifi_manager_is_connected()) {
        lv_obj_clear_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);
    }
    bt_state_t bt = bt_manager_state();
    bool bt_on = (bt == BT_STATE_ADVERTISING || bt == BT_STATE_CONNECTED);
    if (bt_on) {
        lv_obj_clear_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);
    }

    render_steps();
    render_soil();

    /* Clima: refresca (auto-limitado) y refleja la caché compartida */
    weather_service_refresh(false);
    uint32_t gen = weather_service_generation();
    if (gen != s_wx_gen && s_wx_emoji && s_wx_temp) {
        s_wx_gen = gen;
        weather_data_t w;
        if (weather_service_get(&w) && w.valid) {
            lv_label_set_text(s_wx_emoji, w.emoji);
            lv_label_set_text(s_wx_temp, w.temp);
            lv_obj_clear_flag(s_wx_emoji, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_wx_temp, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void ui_watchface_show(void)
{
    if (s_wf) return;
    s_wx_gen = 0xFFFFFFFF; /* forzar refresco del clima desde caché al mostrar */

    s_wf = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_wf);
    lv_obj_set_size(s_wf, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_wf, lv_color_hex(0x000814), 0);
    lv_obj_set_style_bg_opa(s_wf, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_wf, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_wf, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_wf, wf_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(s_wf, wf_gesture_cb, LV_EVENT_GESTURE, NULL);

    /* Arco de segundos pegado al borde circular */
    s_arc = lv_arc_create(s_wf);
    lv_obj_set_size(s_arc, 236, 236);
    lv_obj_center(s_arc);
    lv_arc_set_rotation(s_arc, 270); /* 0 s arriba */
    lv_arc_set_bg_angles(s_arc, 0, 360);
    lv_arc_set_range(s_arc, 0, 60);
    lv_obj_remove_style(s_arc, NULL, LV_PART_KNOB);       /* sin perilla */
    lv_obj_clear_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);      /* solo decorativo */
    lv_obj_set_style_arc_width(s_arc, 4, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc, lv_color_hex(0x0E2436), LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_arc, lv_color_hex(0x2E82C8), LV_PART_INDICATOR);

    /* Clima (arriba): emoji + temperatura, ocultos hasta tener dato */
    s_wx_emoji = lv_label_create(s_wf);
    lv_obj_set_style_text_font(s_wx_emoji, &font_weather_28, 0);
    lv_obj_set_style_text_color(s_wx_emoji, lv_color_white(), 0);
    lv_label_set_text(s_wx_emoji, "");
    lv_obj_align(s_wx_emoji, LV_ALIGN_TOP_MID, -26, 40);
    lv_obj_add_flag(s_wx_emoji, LV_OBJ_FLAG_HIDDEN);

    s_wx_temp = lv_label_create(s_wf);
    lv_obj_set_style_text_font(s_wx_temp, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_wx_temp, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_wx_temp, "");
    lv_obj_align(s_wx_temp, LV_ALIGN_TOP_MID, 16, 46);
    lv_obj_add_flag(s_wx_temp, LV_OBJ_FLAG_HIDDEN);

    /* Hora grande */
    s_time_label = lv_label_create(s_wf);
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_time_label, lv_color_white(), 0);
    lv_label_set_text(s_time_label, "--:--");
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, -14);

    /* Fecha */
    s_date_label = lv_label_create(s_wf);
    lv_obj_set_style_text_font(s_date_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x8FA8C8), 0);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, 28);

    /* Iconos de estado: arriba, como barra de estado, para dejar el borde
     * inferior libre para los datos. */
    s_wifi_icon = lv_label_create(s_wf);
    lv_label_set_text(s_wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_wifi_icon, lv_color_hex(0x8FA8C8), 0);
    lv_obj_align(s_wifi_icon, LV_ALIGN_TOP_MID, -14, 20);

    s_bt_icon = lv_label_create(s_wf);
    lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(s_bt_icon, lv_color_hex(0x8FA8C8), 0);
    lv_obj_align(s_bt_icon, LV_ALIGN_TOP_MID, 14, 20);

    /* Fila inferior: pasos y planta. */
    s_steps_lbl = lv_label_create(s_wf);
    lv_obj_set_style_text_font(s_steps_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_steps_lbl, lv_color_hex(0x35D07F), 0);
    lv_label_set_text(s_steps_lbl, "0");
    lv_obj_align(s_steps_lbl, LV_ALIGN_CENTER, -42, 62);

    s_soil_lbl = lv_label_create(s_wf);
    lv_obj_set_style_text_font(s_soil_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_soil_lbl, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_soil_lbl, "");
    lv_obj_align(s_soil_lbl, LV_ALIGN_CENTER, 34, 62);
    lv_obj_add_flag(s_soil_lbl, LV_OBJ_FLAG_HIDDEN);

    wf_tick_cb(NULL);
    s_timer = lv_timer_create(wf_tick_cb, 1000, NULL);
}
