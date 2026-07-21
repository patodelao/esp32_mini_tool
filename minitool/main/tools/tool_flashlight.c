/*
 * tool_flashlight.c — Linterna: la pantalla completa como fuente de luz.
 * Tocar cicla los modos: blanco, cálido, rojo (visión nocturna), verde, azul.
 */
#include "tool.h"

static const lv_color_t k_colors[] = {
    LV_COLOR_MAKE(0xFF, 0xFF, 0xFF), /* blanco */
    LV_COLOR_MAKE(0xFF, 0xD9, 0x9A), /* cálido */
    LV_COLOR_MAKE(0xC8, 0x1E, 0x1E), /* rojo */
    LV_COLOR_MAKE(0x20, 0xC0, 0x50), /* verde */
    LV_COLOR_MAKE(0x2E, 0x82, 0xC8), /* azul */
};
#define N_COLORS (sizeof(k_colors) / sizeof(k_colors[0]))

static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_hint = NULL;
static int s_mode = 0;

static void apply_mode(void)
{
    lv_obj_set_style_bg_color(s_panel, k_colors[s_mode], 0);
}

static void tap_cb(lv_event_t *e)
{
    (void)e;
    s_mode = (s_mode + 1) % N_COLORS;
    apply_mode();
    if (s_hint) {
        lv_obj_del(s_hint); /* la pista solo se muestra hasta el primer toque */
        s_hint = NULL;
    }
}

static void flashlight_open(lv_obj_t *parent)
{
    s_panel = lv_obj_create(parent);
    lv_obj_remove_style_all(s_panel);
    lv_obj_set_size(s_panel, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_panel);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_panel, tap_cb, LV_EVENT_CLICKED, NULL);
    apply_mode();

    s_hint = lv_label_create(s_panel);
    lv_label_set_text(s_hint, "toca para cambiar color");
    lv_obj_set_style_text_color(s_hint, lv_color_hex(0x333333), 0);
    lv_obj_align(s_hint, LV_ALIGN_CENTER, 0, 70);
}

static void flashlight_close(void)
{
    s_panel = NULL;
    s_hint = NULL;
}

const tool_t tool_flashlight = {
    .name = "Luz",
    .icon = LV_SYMBOL_CHARGE,
    .accent = 0xFFE05A,
    .open = flashlight_open,
    .close = flashlight_close,
};
