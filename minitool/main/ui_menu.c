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

static void open_selected_tool(lv_event_t *e)
{
    (void)e;
    if (!s_menu_roller) return;

    uint16_t selected = lv_roller_get_selected(s_menu_roller);
    if (selected >= (uint16_t)g_tools_count) return;

    const tool_t *tool = g_tools[selected];
    if (!tool) return;

    ESP_LOGI(TAG, "Abriendo herramienta: %s", tool->name ? tool->name : "(sin nombre)");

    lv_obj_clean(lv_scr_act());
    s_menu_roller = NULL;
    s_current_tool = tool;

    if (tool->open) {
        tool->open(lv_scr_act());
    }
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
    lv_obj_clean(lv_scr_act());

    build_menu_options();

    s_menu_roller = lv_roller_create(lv_scr_act());
    lv_roller_set_options(s_menu_roller, s_menu_options, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(s_menu_roller, 3);
    lv_obj_set_width(s_menu_roller, 220);
    lv_obj_center(s_menu_roller);
    lv_obj_add_event_cb(s_menu_roller, open_selected_tool, LV_EVENT_CLICKED, NULL);

    lv_obj_set_style_bg_opa(s_menu_roller, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_menu_roller, LV_OPA_TRANSP, LV_PART_SELECTED);
    lv_obj_set_style_border_width(s_menu_roller, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_menu_roller, 0, LV_PART_SELECTED);

    lv_obj_set_style_text_font(s_menu_roller, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_menu_roller, lv_color_hex(0x556677), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_menu_roller, &lv_font_montserrat_28, LV_PART_SELECTED);
    lv_obj_set_style_text_color(s_menu_roller, lv_color_white(), LV_PART_SELECTED);

    lv_obj_set_style_text_align(s_menu_roller, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_text_align(s_menu_roller, LV_TEXT_ALIGN_LEFT, LV_PART_SELECTED);
}

void ui_menu_show(void)
{
    create_main_menu();
}
