/*
 * ui_menu.c — Carrusel de herramientas para la pantalla redonda 240x240.
 *
 * Estados: o bien se muestra el menú (s_menu), o bien una herramienta abierta
 * a pantalla completa (s_tool_screen). Nunca ambos a la vez.
 */
#include "ui_menu.h"
#include "tool.h"

#include "esp_log.h"

static const char *TAG = "menu";

static lv_obj_t *s_menu = NULL;         /* contenedor del carrusel */
static lv_obj_t *s_tool_screen = NULL;  /* contenedor de la tool abierta */
static const tool_t *s_current = NULL;  /* tool actualmente abierta */

/* Cierra la tool abierta (si hay) y vuelve al menú. */
static void close_tool_and_show_menu(void)
{
    if (s_current && s_current->close) {
        s_current->close();
    }
    s_current = NULL;
    if (s_tool_screen) {
        lv_obj_del(s_tool_screen);
        s_tool_screen = NULL;
    }
    ui_menu_show();
}

static void back_event_cb(lv_event_t *e)
{
    (void)e;
    close_tool_and_show_menu();
}

/* Gesto global: swipe a la derecha dentro de una tool = volver al menú. */
static void screen_gesture_cb(lv_event_t *e)
{
    (void)e;
    if (!s_current) return; /* en el menú, el swipe es scroll del carrusel */
    if (lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        close_tool_and_show_menu();
    }
}

/* Abre la tool asociada al tile pulsado. */
static void tile_event_cb(lv_event_t *e)
{
    const tool_t *tool = (const tool_t *)lv_event_get_user_data(e);
    if (!tool) return;

    ESP_LOGI(TAG, "Abriendo herramienta: %s", tool->name);

    if (s_menu) {
        lv_obj_del(s_menu);
        s_menu = NULL;
    }

    /* Contenedor a pantalla completa para la herramienta */
    s_tool_screen = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_tool_screen);
    lv_obj_set_size(s_tool_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_tool_screen);
    lv_obj_clear_flag(s_tool_screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_tool_screen, lv_color_hex(0x001B2E), 0);
    lv_obj_set_style_bg_opa(s_tool_screen, LV_OPA_COVER, 0);

    s_current = tool;
    if (tool->open) {
        tool->open(s_tool_screen);
    }

    /* Botón de volver (esquina superior, dentro del círculo visible).
     * FLOATING: queda fuera de cualquier layout flex/grid que use la tool. */
    lv_obj_t *back = lv_btn_create(s_tool_screen);
    lv_obj_add_flag(back, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(back, 34, 34);
    lv_obj_align(back, LV_ALIGN_TOP_MID, 0, 6);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_lbl = lv_label_create(back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_center(back_lbl);
}

/* Crea un tile (botón redondo con icono + nombre) para una herramienta. */
static void create_tile(lv_obj_t *parent, const tool_t *tool)
{
    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_set_size(tile, 150, 150);
    lv_obj_set_style_radius(tile, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x0E3A5F), 0);
    lv_obj_set_style_bg_grad_color(tile, lv_color_hex(0x072033), 0);
    lv_obj_set_style_bg_grad_dir(tile, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(tile, 2, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(0x2E82C8), 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(tile, tile_event_cb, LV_EVENT_CLICKED, (void *)tool);

    /* Layout vertical centrado: icono grande + nombre */
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *icon = lv_label_create(tile);
    lv_label_set_text(icon, tool->icon ? tool->icon : LV_SYMBOL_DUMMY);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(icon, lv_color_white(), 0);

    lv_obj_t *name = lv_label_create(tile);
    lv_label_set_text(name, tool->name);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(name, lv_color_hex(0xBFE0FF), 0);
}

void ui_menu_show(void)
{
    /* Registrar el handler de gestos una sola vez, a nivel de pantalla */
    static bool s_gesture_registered = false;
    if (!s_gesture_registered) {
        lv_obj_add_event_cb(lv_scr_act(), screen_gesture_cb, LV_EVENT_GESTURE, NULL);
        s_gesture_registered = true;
    }

    /* Fondo del menú: gradiente azul ocupando toda la pantalla redonda */
    s_menu = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_menu);
    lv_obj_set_size(s_menu, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_menu);
    lv_obj_set_style_bg_color(s_menu, lv_color_hex(0x00CFFF), 0);
    lv_obj_set_style_bg_grad_color(s_menu, lv_color_hex(0x00243D), 0);
    lv_obj_set_style_bg_grad_dir(s_menu, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(s_menu, LV_OPA_COVER, 0);

    /* Carrusel horizontal con snap al centro */
    lv_obj_set_flex_flow(s_menu, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_menu, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_snap_x(s_menu, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_dir(s_menu, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(s_menu, LV_SCROLLBAR_MODE_OFF);
    /* Padding lateral para que el primer/último tile pueda centrarse */
    lv_obj_set_style_pad_left(s_menu, 45, 0);
    lv_obj_set_style_pad_right(s_menu, 45, 0);
    lv_obj_set_style_pad_column(s_menu, 20, 0);

    for (int i = 0; i < g_tools_count; i++) {
        create_tile(s_menu, g_tools[i]);
    }
}
