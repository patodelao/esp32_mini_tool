/*
 * tool_sensors.c — Visor de sensores del home-lab (MQTT).
 *
 * Muestra el valor actual y un gráfico del histórico del sensor seleccionado.
 * El botón inferior cicla entre los sensores vistos por sensor_service.
 */
#include "tool.h"
#include "sensor_service.h"

#include <stdio.h>
#include <math.h>

static lv_obj_t *s_id_lbl = NULL;
static lv_obj_t *s_val_lbl = NULL;
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser = NULL;
static lv_obj_t *s_idx_lbl = NULL;
static lv_obj_t *s_empty = NULL;
static lv_timer_t *s_poll = NULL;
static int s_sel = 0;

static void refresh(void)
{
    int n = sensor_count();

    if (n == 0) {
        if (s_empty) lv_obj_clear_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
        if (s_chart) lv_obj_add_flag(s_chart, LV_OBJ_FLAG_HIDDEN);
        if (s_val_lbl) lv_label_set_text(s_val_lbl, "");
        if (s_id_lbl) lv_label_set_text(s_id_lbl, "Sensores");
        if (s_idx_lbl) lv_label_set_text(s_idx_lbl, "");
        return;
    }
    if (s_empty) lv_obj_add_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
    if (s_chart) lv_obj_clear_flag(s_chart, LV_OBJ_FLAG_HIDDEN);

    if (s_sel >= n) s_sel = 0;

    char id[32], val[16];
    sensor_get(s_sel, id, sizeof(id), val, sizeof(val), NULL);
    if (s_id_lbl) lv_label_set_text(s_id_lbl, id);
    if (s_val_lbl) lv_label_set_text(s_val_lbl, val);

    char ib[16];
    snprintf(ib, sizeof(ib), "%d/%d", s_sel + 1, n);
    if (s_idx_lbl) lv_label_set_text(s_idx_lbl, ib);

    /* Histórico -> gráfico */
    float h[SENSOR_HIST];
    int hn = sensor_history(s_sel, h, SENSOR_HIST);
    if (hn > 0 && s_chart && s_ser) {
        float mn = h[0], mx = h[0];
        for (int i = 1; i < hn; i++) { if (h[i] < mn) mn = h[i]; if (h[i] > mx) mx = h[i]; }
        int vmin = (int)floorf(mn * 10.0f);
        int vmax = (int)ceilf(mx * 10.0f);
        if (vmax - vmin < 2) { vmin -= 5; vmax += 5; } /* evitar rango plano */
        lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, vmin, vmax);
        lv_chart_set_point_count(s_chart, hn);
        for (int i = 0; i < hn; i++) {
            lv_chart_set_value_by_id(s_chart, s_ser, i, (int)lroundf(h[i] * 10.0f));
        }
        lv_chart_refresh(s_chart);
    }
}

static void poll_cb(lv_timer_t *t) { (void)t; refresh(); }

static void next_cb(lv_event_t *e)
{
    (void)e;
    int n = sensor_count();
    if (n > 0) s_sel = (s_sel + 1) % n;
    refresh();
}

static void sensors_open(lv_obj_t *parent)
{
    s_sel = 0;

    s_id_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_id_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_id_lbl, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_id_lbl, "Sensores");
    lv_obj_align(s_id_lbl, LV_ALIGN_TOP_MID, 0, 24);

    s_val_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_val_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_val_lbl, lv_color_white(), 0);
    lv_label_set_text(s_val_lbl, "");
    lv_obj_align(s_val_lbl, LV_ALIGN_TOP_MID, 0, 48);

    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, 176, 70);
    lv_obj_align(s_chart, LV_ALIGN_CENTER, 0, 18);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(s_chart, 3, 0);
    lv_chart_set_update_mode(s_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_bg_opa(s_chart, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_size(s_chart, 0, LV_PART_INDICATOR); /* sin puntos */
    lv_obj_set_style_line_color(s_chart, lv_color_hex(0x2A3A48), LV_PART_MAIN);
    s_ser = lv_chart_add_series(s_chart, lv_color_hex(0x35D07F), LV_CHART_AXIS_PRIMARY_Y);

    s_idx_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_idx_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_idx_lbl, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_idx_lbl, "");
    lv_obj_align(s_idx_lbl, LV_ALIGN_BOTTOM_MID, 0, -58);

    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 104, 36);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_set_style_radius(btn, 18, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(btn, next_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, LV_SYMBOL_RIGHT " Sensor");
    lv_obj_center(bl);

    s_empty = lv_label_create(parent);
    lv_obj_set_style_text_font(s_empty, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_empty, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_empty, "Sin sensores\nlabo/sensor/<id>");
    lv_obj_set_style_text_align(s_empty, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_empty, LV_ALIGN_CENTER, 0, 0);

    refresh();
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void sensors_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    s_id_lbl = s_val_lbl = s_chart = s_idx_lbl = s_empty = NULL;
    s_ser = NULL;
}

const tool_t tool_sensors = {
    .name = "Sensores",
    .icon = LV_SYMBOL_EYE_OPEN,
    .accent = 0x35D07F,
    .open = sensors_open,
    .close = sensors_close,
};
