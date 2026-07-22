/*
 * ui_menu.c — Menú principal giratorio (roller) para pantalla redonda.
 */
#include "ui_menu.h"

#include <stdio.h>

#include "esp_log.h"
#include "tool.h"
#include "ui_watchface.h"

static const char *TAG = "menu";

#define IDLE_TIMEOUT_MS 20000
#define MENU_OPTIONS_BUF_SIZE 768

static lv_obj_t *s_menu_roller = NULL;
static const tool_t *s_current_tool = NULL;
static lv_timer_t *s_idle_timer = NULL;
static char s_menu_options[MENU_OPTIONS_BUF_SIZE];

/* Índice de tool pendiente de abrir tras el flash táctil (-1 = ninguno) */
static int s_pending_open = -1;
static lv_timer_t *s_open_timer = NULL;

#define ACCENT_DEFAULT 0x2E82C8

static uint32_t tool_accent(int idx)
{
    if (idx < 0 || idx >= g_tools_count) return ACCENT_DEFAULT;
    const tool_t *t = g_tools[idx];
    return (t && t->accent) ? t->accent : ACCENT_DEFAULT;
}

/* Tiñe el resaltado (pill + texto) con el acento de la tool seleccionada. */
static void apply_accent(uint16_t selected, lv_opa_t pill_opa)
{
    if (!s_menu_roller) return;
    lv_color_t accent = lv_color_hex(tool_accent(selected));
    lv_obj_set_style_bg_color(s_menu_roller, accent, LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(s_menu_roller, pill_opa, LV_PART_SELECTED);
}

static void close_current_tool(void)
{
    if (s_current_tool && s_current_tool->close) {
        s_current_tool->close();
    }
    s_current_tool = NULL;
}

static void idle_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_current_tool || !s_menu_roller || ui_watchface_active()) return;
    if (lv_disp_get_inactive_time(NULL) > IDLE_TIMEOUT_MS) {
        ui_watchface_show();
    }
}

static void build_menu_options(void)
{
    size_t used = 0;
    s_menu_options[0] = '\0';

    for (int i = 0; i < g_tools_count; i++) {
        const tool_t *tool = g_tools[i];
        const char *icon = (tool && tool->icon) ? tool->icon : LV_SYMBOL_DUMMY;
        const char *name = (tool && tool->name) ? tool->name : "Tool";
        const char *sep = (i == (g_tools_count - 1)) ? "" : "\n";
        int written = lv_snprintf(
            s_menu_options + used,
            MENU_OPTIONS_BUF_SIZE - used,
            "%s  %s%s",
            icon,
            name,
            sep
        );
        if (written <= 0 || (size_t)written >= (MENU_OPTIONS_BUF_SIZE - used)) {
            break;
        }
        used += (size_t)written;
    }
}

static void do_open_tool(int idx)
{
    if (idx < 0 || idx >= g_tools_count) return;
    const tool_t *tool = g_tools[idx];
    if (!tool) return;

    ESP_LOGI(TAG, "Abriendo herramienta: %s", tool->name ? tool->name : "(sin nombre)");

    lv_obj_clean(lv_scr_act());
    s_menu_roller = NULL;
    s_current_tool = tool;

    if (tool->open) {
        tool->open(lv_scr_act());
    }
}

/* Se dispara al terminar el flash táctil: abre la tool pendiente. */
static void open_after_flash_cb(lv_timer_t *t)
{
    (void)t;
    s_open_timer = NULL;
    int idx = s_pending_open;
    s_pending_open = -1;
    do_open_tool(idx);
}

/* Recolorea el resaltado con el acento de la tool al cambiar de selección. */
static void roller_value_changed(lv_event_t *e)
{
    (void)e;
    if (!s_menu_roller) return;
    apply_accent(lv_roller_get_selected(s_menu_roller), LV_OPA_30);
}

static void open_selected_tool(lv_event_t *e)
{
    (void)e;
    if (!s_menu_roller || s_pending_open >= 0) return; /* ya hay un flash en curso */

    uint16_t selected = lv_roller_get_selected(s_menu_roller);
    if (selected >= (uint16_t)g_tools_count) return;

    /* Feedback táctil: destello del acento y apertura tras un breve instante */
    apply_accent(selected, LV_OPA_COVER);
    s_pending_open = (int)selected;
    s_open_timer = lv_timer_create(open_after_flash_cb, 130, NULL);
    lv_timer_set_repeat_count(s_open_timer, 1);
}

static void screen_gesture_cb(lv_event_t *e)
{
    (void)e;
    if (!s_current_tool) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;

    if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
        close_current_tool();
        lv_obj_clean(lv_scr_act());
        create_main_menu();
    }
}

void create_main_menu(void)
{
    static bool s_registered = false;
    if (!s_registered) {
        lv_obj_add_event_cb(lv_scr_act(), screen_gesture_cb, LV_EVENT_GESTURE, NULL);
        s_idle_timer = lv_timer_create(idle_tick_cb, 1000, NULL);
        s_registered = true;
    } else if (!s_idle_timer) {
        s_idle_timer = lv_timer_create(idle_tick_cb, 1000, NULL);
    }

    close_current_tool();
    if (s_open_timer) { lv_timer_del(s_open_timer); s_open_timer = NULL; }
    s_pending_open = -1;
    lv_obj_clean(lv_scr_act());

    build_menu_options();

    s_menu_roller = lv_roller_create(lv_scr_act());
    lv_roller_set_options(s_menu_roller, s_menu_options, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(s_menu_roller, 3);
    lv_obj_set_width(s_menu_roller, 220);
    lv_obj_center(s_menu_roller);
    lv_obj_add_event_cb(s_menu_roller, open_selected_tool, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(s_menu_roller, roller_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* Animación de asentado suave al soltar el scroll */
    lv_obj_set_style_anim_time(s_menu_roller, 280, LV_PART_MAIN);

    /* Fondo transparente; el resaltado es una "pill" redondeada con acento */
    lv_obj_set_style_bg_opa(s_menu_roller, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_menu_roller, 0, LV_PART_MAIN);

    lv_obj_set_style_border_width(s_menu_roller, 0, LV_PART_SELECTED);
    lv_obj_set_style_radius(s_menu_roller, 22, LV_PART_SELECTED);
    lv_obj_set_style_pad_left(s_menu_roller, 14, LV_PART_SELECTED);

    lv_obj_set_style_text_font(s_menu_roller, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_menu_roller, lv_color_hex(0x556677), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_menu_roller, &lv_font_montserrat_28, LV_PART_SELECTED);
    lv_obj_set_style_text_color(s_menu_roller, lv_color_white(), LV_PART_SELECTED);

    lv_obj_set_style_text_align(s_menu_roller, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_text_align(s_menu_roller, LV_TEXT_ALIGN_LEFT, LV_PART_SELECTED);

    /* Acento inicial de la tool centrada */
    apply_accent(lv_roller_get_selected(s_menu_roller), LV_OPA_30);
}

void ui_menu_show(void)
{
    create_main_menu();
}
