/*
 * tool_weather.c — Vista del clima actual.
 *
 * La descarga y la caché viven en weather_service (compartido con el
 * screensaver). Esta tool solo muestra el dato cacheado, dispara refrescos y
 * refleja el estado leyendo la caché por polling.
 */
#include "tool.h"
#include "wifi_manager.h"
#include "weather_service.h"

#include "lvgl.h"

/* Fuentes generadas */
LV_FONT_DECLARE(font_weather_28);
LV_FONT_DECLARE(font_text_16);

static lv_obj_t *s_city_label = NULL;
static lv_obj_t *s_temp_label = NULL;
static lv_obj_t *s_emoji_label = NULL;
static lv_obj_t *s_desc_label = NULL;
static lv_obj_t *s_refresh_btn = NULL;
static lv_timer_t *s_poll = NULL;
static uint32_t s_seen_gen = 0xFFFFFFFF;

static void apply_cache(void) {
    weather_data_t w;
    weather_service_get(&w);
    if (s_city_label)  lv_label_set_text(s_city_label,  w.city[0]  ? w.city  : "Sin datos");
    if (s_temp_label)  lv_label_set_text(s_temp_label,  w.temp);
    if (s_emoji_label) lv_label_set_text(s_emoji_label, w.emoji);
    if (s_desc_label)  lv_label_set_text(s_desc_label,  w.desc[0] ? w.desc : "Toca refrescar");
}

static void poll_cb(lv_timer_t *t) {
    (void)t;
    uint32_t gen = weather_service_generation();
    bool fetching = weather_service_is_fetching();

    if (fetching && s_city_label) {
        lv_label_set_text(s_city_label, "Actualizando...");
    }
    if (gen != s_seen_gen) {
        s_seen_gen = gen;
        apply_cache();
    }
}

static void refresh_click_cb(lv_event_t *e) {
    (void)e;
    if (!wifi_manager_is_connected()) {
        if (s_city_label) lv_label_set_text(s_city_label, "Sin Wi-Fi");
        return;
    }
    weather_service_refresh(true);
}

static void weather_open(lv_obj_t *parent) {
    s_seen_gen = 0xFFFFFFFF;

    s_city_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_city_label, &font_text_16, 0);
    lv_obj_set_style_text_color(s_city_label, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_city_label, "Esperando red...");
    lv_obj_align(s_city_label, LV_ALIGN_TOP_MID, 0, 30);

    s_temp_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_temp_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_temp_label, lv_color_white(), 0);
    lv_label_set_text(s_temp_label, "--.- \xC2\xB0""C");
    lv_obj_center(s_temp_label);

    lv_obj_t *desc_cont = lv_obj_create(parent);
    lv_obj_remove_style_all(desc_cont);
    lv_obj_set_size(desc_cont, 200, 40);
    lv_obj_align(desc_cont, LV_ALIGN_CENTER, 0, 45);
    lv_obj_set_flex_flow(desc_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(desc_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(desc_cont, 8, 0);

    s_emoji_label = lv_label_create(desc_cont);
    lv_obj_set_style_text_font(s_emoji_label, &font_weather_28, 0);
    lv_obj_set_style_text_color(s_emoji_label, lv_color_white(), 0);
    lv_label_set_text(s_emoji_label, "☁️");

    s_desc_label = lv_label_create(desc_cont);
    lv_obj_set_style_text_font(s_desc_label, &font_text_16, 0);
    lv_obj_set_style_text_color(s_desc_label, lv_color_hex(0x35D07F), 0);
    lv_label_set_text(s_desc_label, "Toca refrescar");

    s_refresh_btn = lv_btn_create(parent);
    lv_obj_set_size(s_refresh_btn, 130, 36);
    lv_obj_align(s_refresh_btn, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_radius(s_refresh_btn, 18, 0);
    lv_obj_set_style_bg_color(s_refresh_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_refresh_btn, refresh_click_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_lbl = lv_label_create(s_refresh_btn);
    lv_label_set_text(btn_lbl, LV_SYMBOL_REFRESH " Refrescar");
    lv_obj_center(btn_lbl);

    /* Mostrar de inmediato lo que haya en caché y refrescar si toca */
    apply_cache();
    s_seen_gen = weather_service_generation();
    s_poll = lv_timer_create(poll_cb, 500, NULL);

    if (wifi_manager_is_connected()) {
        weather_service_refresh(false);
    }
}

static void weather_close(void) {
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
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
