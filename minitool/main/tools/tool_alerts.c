/*
 * tool_alerts.c — Historial de notificaciones.
 *
 * Un toast dura unos segundos y se va: si no estabas mirando la pantalla, el
 * aviso se perdió. Acá quedan las últimas NOTIFY_HIST, más recientes arriba,
 * con su hora y el color de su nivel.
 *
 * Registra TODO lo que pasó por ui_notify_push: alertas del bus MQTT (nodos,
 * refri), umbrales locales de sensores y confirmaciones de la tool Control.
 *
 * El botón de papelera vacía la lista (dos toques, como en Sensores). El
 * historial se persiste en NVS, así que sobrevive a reinicios y cortes de luz
 * — que es justo cuando querés saber qué pasó.
 *
 * OJO: acá está el HISTORIAL. Los umbrales de cada sensor se editan en la tool
 * Sensores, tocando el sensor y después el engranaje.
 */
#include "tool.h"
#include "ui_theme.h"
#include "ui_notify.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static lv_obj_t   *s_list = NULL;
static lv_obj_t   *s_empty = NULL;
static lv_obj_t   *s_clear_btn = NULL;
static lv_obj_t   *s_clear_lbl = NULL;
static lv_timer_t *s_poll = NULL;
static lv_timer_t *s_confirm_tmr = NULL;
static bool        s_confirm = false;
static int         s_shown = -1;   /* cuántas había en el último dibujado */

static const char *level_color(notify_level_t lv)
{
    switch (lv) {
        case NOTIFY_SUCCESS: return "35D07F";
        case NOTIFY_WARNING: return "F1C40F";
        case NOTIFY_ALERT:   return "E74C3C";
        case NOTIFY_INFO:
        default:             return "3498DB";
    }
}

/* "14:32" con la hora local, o "--:--" si el reloj no estaba sincronizado. */
static void fmt_time(time_t ts, char *out, int out_size)
{
    if (ts <= 0) { snprintf(out, out_size, "--:--"); return; }
    struct tm tm;
    localtime_r(&ts, &tm);
    snprintf(out, out_size, "%02d:%02d", tm.tm_hour, tm.tm_min);
}

/* Una alerta = tarjeta con: fila superior [punto de nivel · origen ··· hora] y
 * el mensaje debajo, envuelto. El punto y el origen toman el color del nivel. */
static void add_card(const notify_record_t *r)
{
    lv_obj_t *card = lv_obj_create(s_list);
    lv_obj_remove_style_all(card);
    lv_obj_set_width(card, 190);
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_all(card, 8, 0);
    lv_obj_set_style_pad_row(card, 2, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_color_t lc = lv_color_hex((uint32_t)strtoul(level_color(r->level), NULL, 16));

    /* Fila superior: punto + origen + (empuja) + hora. */
    lv_obj_t *top = lv_obj_create(card);
    lv_obj_remove_style_all(top);
    lv_obj_set_width(top, LV_PCT(100));
    lv_obj_set_height(top, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(top, 6, 0);
    lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dot = lv_obj_create(top);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 10, 10);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(dot, lc, 0);

    lv_obj_t *src = lv_label_create(top);
    lv_obj_set_style_text_font(src, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(src, lc, 0);
    lv_label_set_text(src, r->source);
    lv_obj_set_flex_grow(src, 1);          /* empuja la hora a la derecha */

    char hora[8];
    fmt_time(r->ts, hora, sizeof(hora));
    lv_obj_t *tl = lv_label_create(top);
    lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(tl, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(tl, hora);

    /* Mensaje, envuelto. */
    lv_obj_t *msg = lv_label_create(card);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, LV_PCT(100));
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(msg, lv_color_hex(0xDDE6F0), 0);
    lv_label_set_text(msg, r->msg);
}

static void rebuild(void)
{
    lv_obj_clean(s_list);

    int n = ui_notify_history_count();
    for (int i = 0; i < n; i++) {
        notify_record_t r;
        if (!ui_notify_history_get(i, &r)) continue;
        add_card(&r);
    }

    if (s_empty) {
        if (n == 0) lv_obj_clear_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
        else        lv_obj_add_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
    }
    s_shown = n;
}

/* Solo redibuja si llegó algo nuevo: reconstruir la lista en cada tick haría
 * imposible desplazarla con el dedo. */
static void poll_cb(lv_timer_t *t)
{
    (void)t;
    if (ui_notify_history_count() != s_shown) rebuild();
}

/* --------------------------- Borrado (2 toques) --------------------------- */

static void clear_visual_idle(void)
{
    if (s_clear_btn) lv_obj_set_style_bg_color(s_clear_btn, lv_color_hex(0x33445A), 0);
    if (s_clear_lbl) lv_label_set_text(s_clear_lbl, LV_SYMBOL_TRASH);
}

static void confirm_cancel(void)
{
    if (s_confirm_tmr) { lv_timer_del(s_confirm_tmr); s_confirm_tmr = NULL; }
    s_confirm = false;
    clear_visual_idle();
}

static void confirm_timeout_cb(lv_timer_t *t)
{
    (void)t;
    s_confirm_tmr = NULL;   /* repeat_count 1: LVGL ya lo borra */
    s_confirm = false;
    clear_visual_idle();
}

static void clear_cb(lv_event_t *e)
{
    (void)e;
    if (ui_notify_history_count() == 0) return;
    if (!s_confirm) {
        s_confirm = true;
        if (s_clear_btn) lv_obj_set_style_bg_color(s_clear_btn, lv_color_hex(0xB0403A), 0);
        if (s_clear_lbl) lv_label_set_text(s_clear_lbl, LV_SYMBOL_OK);
        s_confirm_tmr = lv_timer_create(confirm_timeout_cb, 3000, NULL);
        lv_timer_set_repeat_count(s_confirm_tmr, 1);
    } else {
        ui_notify_history_clear();
        confirm_cancel();
        rebuild();
    }
}

/* ----------------------------------- UI ----------------------------------- */

static void alerts_open(lv_obj_t *parent)
{
    s_confirm = false;

    ui_title(parent, "Alertas");

    s_list = lv_obj_create(parent);
    lv_obj_remove_style_all(s_list);
    lv_obj_set_size(s_list, 196, 148);
    lv_obj_align(s_list, LV_ALIGN_CENTER, 0, 4);
    lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_list, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_list, 8, 0);
    lv_obj_set_scroll_dir(s_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_list, LV_SCROLLBAR_MODE_OFF);

    s_empty = lv_label_create(parent);
    lv_obj_set_style_text_font(s_empty, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_empty, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_empty, "Sin alertas");
    lv_obj_align(s_empty, LV_ALIGN_CENTER, 0, 0);

    s_clear_btn = lv_btn_create(parent);
    lv_obj_set_size(s_clear_btn, 44, 32);
    lv_obj_align(s_clear_btn, LV_ALIGN_BOTTOM_MID, 0, -26);
    lv_obj_set_style_radius(s_clear_btn, 16, 0);
    lv_obj_set_style_bg_color(s_clear_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_clear_btn, clear_cb, LV_EVENT_CLICKED, NULL);
    s_clear_lbl = lv_label_create(s_clear_btn);
    lv_label_set_text(s_clear_lbl, LV_SYMBOL_TRASH);
    lv_obj_center(s_clear_lbl);

    s_shown = -1;
    rebuild();
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void alerts_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    if (s_confirm_tmr) { lv_timer_del(s_confirm_tmr); s_confirm_tmr = NULL; }
    s_confirm = false;
    s_list = s_empty = s_clear_btn = s_clear_lbl = NULL;
}

const tool_t tool_alerts = {
    .name = "Alertas",
    .icon = LV_SYMBOL_BELL,
    .accent = 0xE67E22,
    .open = alerts_open,
    .close = alerts_close,
};
