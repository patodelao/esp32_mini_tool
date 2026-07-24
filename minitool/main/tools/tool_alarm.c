/*
 * tool_alarm.c — Vista para editar las alarmas.
 *
 * El despertador en sí vive en alarm_clock (servicio global): esta tool solo
 * muestra y edita. Cerrarla no apaga nada.
 *
 * Una lista con las alarmas: tocar la hora abre el editor, tocar el switch la
 * activa o desactiva. El editor son dos ruedas (hora y minuto), que en una
 * pantalla redonda se manejan mejor que cualquier teclado.
 */
#include "tool.h"
#include "alarm_clock.h"

#include <stdio.h>

static lv_obj_t *s_rows[ALARM_COUNT];
static lv_obj_t *s_sw[ALARM_COUNT];
static lv_obj_t *s_ovl = NULL;
static lv_obj_t *s_roll_h = NULL;
static lv_obj_t *s_roll_m = NULL;
static int       s_editing = -1;

static void refresh_row(int i)
{
    alarm_t a;
    if (!alarm_clock_get(i, &a) || !s_rows[i]) return;

    lv_label_set_text_fmt(s_rows[i], "%02d:%02d", a.hour, a.min);
    /* La hora apagada se ve en gris: el estado tiene que leerse sin buscar el
     * switch. */
    lv_obj_set_style_text_color(s_rows[i],
        lv_color_hex(a.enabled ? 0xDDE6F0 : 0x4A5560), 0);

    if (s_sw[i]) {
        if (a.enabled) lv_obj_add_state(s_sw[i], LV_STATE_CHECKED);
        else           lv_obj_clear_state(s_sw[i], LV_STATE_CHECKED);
    }
}

/* ------------------------------- Editor ---------------------------------- */

static void ovl_close(void)
{
    if (s_ovl) { lv_obj_del(s_ovl); s_ovl = NULL; }
    s_roll_h = s_roll_m = NULL;
    s_editing = -1;
}

static void ovl_ok_cb(lv_event_t *e)
{
    (void)e;
    if (s_editing >= 0 && s_roll_h && s_roll_m) {
        alarm_t a;
        alarm_clock_get(s_editing, &a);
        a.hour = (uint8_t)lv_roller_get_selected(s_roll_h);
        a.min  = (uint8_t)lv_roller_get_selected(s_roll_m);
        a.enabled = true;    /* si la editaste, es porque la querés */
        alarm_clock_set(s_editing, &a);
        refresh_row(s_editing);
    }
    ovl_close();
}

/* Opciones "00\n01\n..." para los rollers. Estáticas: LVGL las referencia. */
static char s_hours[24 * 3 + 1];
static char s_mins[60 * 3 + 1];

static void build_options(void)
{
    int p = 0;
    for (int i = 0; i < 24; i++) p += snprintf(s_hours + p, sizeof(s_hours) - p, i ? "\n%02d" : "%02d", i);
    p = 0;
    for (int i = 0; i < 60; i++) p += snprintf(s_mins + p, sizeof(s_mins) - p, i ? "\n%02d" : "%02d", i);
}

static lv_obj_t *roller(lv_obj_t *parent, const char *opts, int sel, int x)
{
    lv_obj_t *r = lv_roller_create(parent);
    lv_roller_set_options(r, opts, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(r, 3);
    lv_obj_set_width(r, 66);
    lv_obj_align(r, LV_ALIGN_CENTER, x, -14);
    lv_obj_set_style_text_font(r, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(r, lv_color_hex(0x11202E), 0);
    lv_obj_set_style_bg_color(r, lv_color_hex(0x2A4356), LV_PART_SELECTED);
    lv_obj_set_style_border_width(r, 0, 0);
    lv_roller_set_selected(r, sel, LV_ANIM_OFF);
    return r;
}

static void edit_cb(lv_event_t *e)
{
    if (s_ovl) return;
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    alarm_t a;
    if (!alarm_clock_get(i, &a)) return;
    s_editing = i;

    s_ovl = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_ovl);
    lv_obj_set_size(s_ovl, 240, 240);
    lv_obj_center(s_ovl);
    lv_obj_set_style_bg_color(s_ovl, lv_color_hex(0x0A0E12), 0);
    lv_obj_set_style_bg_opa(s_ovl, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_ovl, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *sep = lv_label_create(s_ovl);
    lv_obj_set_style_text_font(sep, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(sep, lv_color_white(), 0);
    lv_label_set_text(sep, ":");
    lv_obj_align(sep, LV_ALIGN_CENTER, 0, -14);

    s_roll_h = roller(s_ovl, s_hours, a.hour, -40);
    s_roll_m = roller(s_ovl, s_mins,  a.min,   40);

    lv_obj_t *ok = lv_btn_create(s_ovl);
    lv_obj_set_size(ok, 120, 38);
    lv_obj_align(ok, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_set_style_radius(ok, 19, 0);
    lv_obj_set_style_bg_color(ok, lv_color_hex(0x35D07F), 0);
    lv_obj_add_event_cb(ok, ovl_ok_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *okl = lv_label_create(ok);
    lv_label_set_text(okl, LV_SYMBOL_OK "  Guardar");
    lv_obj_set_style_text_color(okl, lv_color_hex(0x0A0E12), 0);
    lv_obj_center(okl);
}

static void sw_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t *sw = lv_event_get_target(e);
    alarm_t a;
    if (!alarm_clock_get(i, &a)) return;
    a.enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    alarm_clock_set(i, &a);
    refresh_row(i);
}

/* ---------------------------------- UI ----------------------------------- */

static void alarm_open(lv_obj_t *parent)
{
    build_options();

    lv_obj_t *title = lv_label_create(parent);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(title, "Alarmas");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *list = lv_obj_create(parent);
    lv_obj_remove_style_all(list);
    lv_obj_set_size(list, 196, 168);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 14);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 8, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(list, LV_OBJ_FLAG_EVENT_BUBBLE);

    for (int i = 0; i < ALARM_COUNT; i++) {
        lv_obj_t *card = lv_obj_create(list);
        lv_obj_remove_style_all(card);
        lv_obj_set_size(card, LV_PCT(100), 38);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x1A2733), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(card, 19, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(card, LV_OBJ_FLAG_EVENT_BUBBLE);

        /* La hora es el area pulsable: abre el editor. */
        lv_obj_t *btn = lv_btn_create(card);
        lv_obj_set_size(btn, 86, 34);
        lv_obj_align(btn, LV_ALIGN_LEFT_MID, 4, 0);
        lv_obj_set_style_radius(btn, 17, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x27384A), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_STATE_PRESSED);
        lv_obj_add_event_cb(btn, edit_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        s_rows[i] = lv_label_create(btn);
        lv_obj_set_style_text_font(s_rows[i], &lv_font_montserrat_16, 0);
        lv_obj_center(s_rows[i]);

        s_sw[i] = lv_switch_create(card);
        lv_obj_set_size(s_sw[i], 42, 22);
        lv_obj_align(s_sw[i], LV_ALIGN_RIGHT_MID, -8, 0);
        lv_obj_set_style_bg_color(s_sw[i], lv_color_hex(0x35D07F),
                                  LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_add_event_cb(s_sw[i], sw_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);

        refresh_row(i);
    }
}

static void alarm_close(void)
{
    ovl_close();
    for (int i = 0; i < ALARM_COUNT; i++) { s_rows[i] = NULL; s_sw[i] = NULL; }
}

const tool_t tool_alarm = {
    .name = "Alarmas",
    .icon = LV_SYMBOL_BELL,
    .accent = 0x1E6FC8,
    .open = alarm_open,
    .close = alarm_close,
};
