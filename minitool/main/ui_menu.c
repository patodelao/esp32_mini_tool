/*
 * ui_menu.c — Carrusel de herramientas para la pantalla redonda 240x240.
 *
 * Estados: o bien se muestra el menú (s_menu), o bien una herramienta abierta
 * a pantalla completa (s_tool_screen). Nunca ambos a la vez.
 *
 * Estética:
 *  - Efecto "lupa": el tile centrado se ve a tamaño completo y los laterales
 *    se encogen (transform_zoom según distancia al centro).
 *  - Barra de estado flotante: hora + iconos WiFi/BT según conexión.
 *  - Apertura de tool con animación de deslizamiento vertical.
 *  - Watchface de reposo tras ~20 s sin actividad en el menú.
 */
#include "ui_menu.h"
#include "tool.h"
#include "ui_watchface.h"
#include "wifi_manager.h"
#include "bt_manager.h"

#include <time.h>

#include "esp_log.h"

static const char *TAG = "menu";

#define IDLE_TIMEOUT_MS 20000 /* inactividad antes del watchface */

static lv_obj_t *s_menu = NULL;         /* contenedor del carrusel */
static lv_obj_t *s_tool_screen = NULL;  /* contenedor de la tool abierta */
static const tool_t *s_current = NULL;  /* tool actualmente abierta */

/* Barra de estado del menú */
static lv_obj_t *s_sb_clock = NULL;
static lv_obj_t *s_sb_wifi = NULL;
static lv_obj_t *s_sb_bt = NULL;
static lv_timer_t *s_sb_timer = NULL;

static lv_color_t tool_accent(const tool_t *tool)
{
    return tool->accent ? lv_color_hex(tool->accent) : lv_color_hex(0x2E82C8);
}

/* ------------------------------------------------------- barra de estado */

static void statusbar_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_sb_clock) return;

    time_t now;
    struct tm tm_info;
    time(&now);
    localtime_r(&now, &tm_info);
    char buf[8];
    strftime(buf, sizeof(buf), "%H:%M", &tm_info);
    lv_label_set_text(s_sb_clock, buf);

    /* Presencia = conectado: el icono aparece solo cuando hay conexión */
    if (wifi_manager_is_connected()) {
        lv_obj_clear_flag(s_sb_wifi, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_sb_wifi, LV_OBJ_FLAG_HIDDEN);
    }
    bt_state_t bt = bt_manager_state();
    bool bt_on = (bt == BT_STATE_ADVERTISING || bt == BT_STATE_CONNECTED);
    if (bt_on) {
        lv_obj_clear_flag(s_sb_bt, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_sb_bt, LV_OBJ_FLAG_HIDDEN);
    }
}

static void statusbar_create(lv_obj_t *parent)
{
    s_sb_wifi = lv_label_create(parent);
    lv_obj_add_flag(s_sb_wifi, LV_OBJ_FLAG_FLOATING);
    lv_label_set_text(s_sb_wifi, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_sb_wifi, lv_color_white(), 0);
    lv_obj_align(s_sb_wifi, LV_ALIGN_TOP_MID, -34, 10);

    s_sb_clock = lv_label_create(parent);
    lv_obj_add_flag(s_sb_clock, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_text_color(s_sb_clock, lv_color_white(), 0);
    lv_label_set_text(s_sb_clock, "--:--");
    lv_obj_align(s_sb_clock, LV_ALIGN_TOP_MID, 0, 10);

    s_sb_bt = lv_label_create(parent);
    lv_obj_add_flag(s_sb_bt, LV_OBJ_FLAG_FLOATING);
    lv_label_set_text(s_sb_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(s_sb_bt, lv_color_white(), 0);
    lv_obj_align(s_sb_bt, LV_ALIGN_TOP_MID, 34, 10);

    statusbar_tick_cb(NULL);
    s_sb_timer = lv_timer_create(statusbar_tick_cb, 1000, NULL);
}

static void statusbar_destroy(void)
{
    if (s_sb_timer) {
        lv_timer_del(s_sb_timer);
        s_sb_timer = NULL;
    }
    /* Los labels los destruye el borrado de s_menu */
    s_sb_clock = s_sb_wifi = s_sb_bt = NULL;
}

/* ------------------------------------------------------- efecto "lupa" */

/*
 * Escala cada tile según su distancia al centro de la pantalla:
 * 256 (100%) centrado, hasta ~176 (69%) en los bordes.
 */
static void apply_magnify(void)
{
    if (!s_menu) return;
    lv_coord_t screen_cx = 120;
    uint32_t n = lv_obj_get_child_cnt(s_menu);
    for (uint32_t i = 0; i < n; i++) {
        lv_obj_t *tile = lv_obj_get_child(s_menu, i);
        if (lv_obj_has_flag(tile, LV_OBJ_FLAG_FLOATING)) continue; /* barra estado */
        lv_area_t coords;
        lv_obj_get_coords(tile, &coords);
        lv_coord_t tile_cx = (coords.x1 + coords.x2) / 2;
        int32_t dist = tile_cx - screen_cx;
        if (dist < 0) dist = -dist;
        int32_t zoom = 256 - (dist * 80) / 120; /* 256 → 176 */
        if (zoom < 176) zoom = 176;
        lv_obj_set_style_transform_zoom(tile, (lv_coord_t)zoom, 0);
    }
}

static void menu_scroll_cb(lv_event_t *e)
{
    (void)e;
    apply_magnify();
}

/* ------------------------------------------------- apertura/cierre tools */

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

static void slide_anim_cb(void *obj, int32_t v)
{
    lv_obj_set_y((lv_obj_t *)obj, v);
}

/* Abre la tool asociada al tile pulsado. */
static void tile_event_cb(lv_event_t *e)
{
    const tool_t *tool = (const tool_t *)lv_event_get_user_data(e);
    if (!tool) return;

    ESP_LOGI(TAG, "Abriendo herramienta: %s", tool->name);

    statusbar_destroy();
    if (s_menu) {
        lv_obj_del(s_menu);
        s_menu = NULL;
    }

    /* Contenedor a pantalla completa para la herramienta */
    s_tool_screen = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_tool_screen);
    lv_obj_set_size(s_tool_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(s_tool_screen, 0, 240); /* parte abajo, sube animado */
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

    /* Animación de entrada: desliza desde abajo */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_tool_screen);
    lv_anim_set_values(&a, 240, 0);
    lv_anim_set_time(&a, 180);
    lv_anim_set_exec_cb(&a, slide_anim_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/* ----------------------------------------------------------------- tiles */

/* Crea un tile (botón redondo con icono + nombre) para una herramienta. */
static void create_tile(lv_obj_t *parent, const tool_t *tool)
{
    lv_color_t accent = tool_accent(tool);

    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_set_size(tile, 150, 150);
    lv_obj_set_style_radius(tile, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x0E3A5F), 0);
    lv_obj_set_style_bg_grad_color(tile, lv_color_hex(0x072033), 0);
    lv_obj_set_style_bg_grad_dir(tile, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(tile, 2, 0);
    lv_obj_set_style_border_color(tile, accent, 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(tile, tile_event_cb, LV_EVENT_CLICKED, (void *)tool);

    /* Layout vertical centrado: icono grande + nombre */
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *icon = lv_label_create(tile);
    lv_label_set_text(icon, tool->icon ? tool->icon : LV_SYMBOL_DUMMY);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(icon, accent, 0);

    lv_obj_t *name = lv_label_create(tile);
    lv_label_set_text(name, tool->name);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(name, lv_color_hex(0xDDEAF6), 0);
}

/* -------------------------------------------------- watchface por reposo */

static void idle_tick_cb(lv_timer_t *t)
{
    (void)t;
    /* Solo desde el menú (no sobre una tool ni sobre un prompt de texto) */
    if (s_current || !s_menu || ui_watchface_active()) return;
    if (lv_disp_get_inactive_time(NULL) > IDLE_TIMEOUT_MS) {
        ui_watchface_show();
    }
}

/* ------------------------------------------------------------------ menú */

void ui_menu_show(void)
{
    /* Registro único de infraestructura global */
    static bool s_registered = false;
    if (!s_registered) {
        lv_obj_add_event_cb(lv_scr_act(), screen_gesture_cb, LV_EVENT_GESTURE, NULL);
        lv_timer_create(idle_tick_cb, 1000, NULL);
        s_registered = true;
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
    /* Un swipe avanza EXACTAMENTE un tile: fácil de elegir sin pasarse */
    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_SCROLL_ONE);
    /* Padding lateral para que el primer/último tile pueda centrarse */
    lv_obj_set_style_pad_left(s_menu, 45, 0);
    lv_obj_set_style_pad_right(s_menu, 45, 0);
    lv_obj_set_style_pad_column(s_menu, 20, 0);
    lv_obj_add_event_cb(s_menu, menu_scroll_cb, LV_EVENT_SCROLL, NULL);

    for (int i = 0; i < g_tools_count; i++) {
        create_tile(s_menu, g_tools[i]);
    }

    statusbar_create(s_menu);

    /* Aplicar la lupa al estado inicial (post-layout) */
    lv_obj_update_layout(s_menu);
    apply_magnify();
}
