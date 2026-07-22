/*
 * ui_watchface.c — Carátula de reposo: hora grande + arco de segundos.
 */
#include "ui_watchface.h"

#include <time.h>

#include "lvgl.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "weather_service.h"

LV_FONT_DECLARE(font_weather_28);

static lv_obj_t *s_wf = NULL;
static lv_obj_t *s_arc = NULL;
static lv_obj_t *s_time_label = NULL;
static lv_obj_t *s_date_label = NULL;
static lv_obj_t *s_wifi_icon = NULL;
static lv_obj_t *s_bt_icon = NULL;
static lv_obj_t *s_wx_emoji = NULL;
static lv_obj_t *s_wx_temp = NULL;
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
}

static void wf_click_cb(lv_event_t *e)
{
    (void)e;
    wf_close();
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
    lv_obj_set_style_text_color(s_wx_temp, lv_color_hex(0x8899AA), 0);
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
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x8899AA), 0);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, 28);

    /* Iconos de estado */
    s_wifi_icon = lv_label_create(s_wf);
    lv_label_set_text(s_wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_wifi_icon, lv_color_white(), 0);
    lv_obj_align(s_wifi_icon, LV_ALIGN_CENTER, -18, 62);

    s_bt_icon = lv_label_create(s_wf);
    lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(s_bt_icon, lv_color_white(), 0);
    lv_obj_align(s_bt_icon, LV_ALIGN_CENTER, 18, 62);

    wf_tick_cb(NULL);
    s_timer = lv_timer_create(wf_tick_cb, 1000, NULL);
}
