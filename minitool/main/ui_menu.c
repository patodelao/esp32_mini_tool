/*
 * ui_menu.c — Menú principal (roller) para pantalla redonda.
 *
 * El roller se desplaza libremente (fluido). Para abrir la herramienta centrada
 * se toca su ícono (el emoji/símbolo a la izquierda del nombre): un objetivo
 * pequeño y deliberado que evita aperturas accidentales al hacer scroll.
 * Arriba hay una barra de estado (reloj + Wi-Fi/BT); abajo, un arco sutil que
 * indica la posición en la lista. Un arco fino enmarca el borde circular.
 */
#include "ui_menu.h"

#include <stdio.h>
#include <time.h>

#include "esp_log.h"
#include "tool.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "ui_watchface.h"

static const char *TAG = "menu";

#define IDLE_TIMEOUT_MS 20000
#define MENU_OPTIONS_BUF_SIZE 768
/* Objetivo de apertura: solo el ícono de la fila central resaltada.
 * X0..X1 = franja horizontal del ícono desde el borde izq. del roller;
 * CENTER_BAND = medio alto de la fila central (excluye filas vecinas). */
#define ICON_ZONE_X0 8
#define ICON_ZONE_X1 52
#define CENTER_BAND  17

static lv_obj_t *s_menu_roller = NULL;
static lv_obj_t *s_pos_arc = NULL;
static lv_obj_t *s_nav_zone = NULL;
static lv_obj_t *s_clock_lbl = NULL;
static lv_obj_t *s_wifi_icon = NULL;
static lv_obj_t *s_bt_icon = NULL;
static const tool_t *s_current_tool = NULL;
static lv_timer_t *s_idle_timer = NULL;
static char s_menu_options[MENU_OPTIONS_BUF_SIZE];

/* Posición del menú a restaurar al volver de una tool (default 0 al encender) */
static uint16_t s_return_pos = 0;

#define ACCENT_DEFAULT 0x2E82C8

static uint32_t pos_accent(int pos)
{
    if (pos < 0 || pos >= g_tools_count) return ACCENT_DEFAULT;
    const tool_t *t = g_tools[pos];
    return (t && t->accent) ? t->accent : ACCENT_DEFAULT;
}

/* Tiñe el resaltado del roller y actualiza el arco de posición con el acento. */
static void apply_accent(uint16_t selected)
{
    lv_color_t accent = lv_color_hex(pos_accent(selected));
    if (s_menu_roller) {
        lv_obj_set_style_bg_color(s_menu_roller, accent, LV_PART_SELECTED);
        lv_obj_set_style_bg_opa(s_menu_roller, LV_OPA_30, LV_PART_SELECTED);
    }
    if (s_pos_arc) {
        lv_obj_set_style_arc_color(s_pos_arc, accent, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(s_pos_arc, accent, LV_PART_KNOB);
        lv_arc_set_value(s_pos_arc, selected);
    }
}

static void close_current_tool(void)
{
    if (s_current_tool && s_current_tool->close) {
        s_current_tool->close();
    }
    s_current_tool = NULL;
}

static void update_statusbar(void)
{
    if (s_clock_lbl) {
        time_t now;
        struct tm ti;
        time(&now);
        localtime_r(&now, &ti);
        char buf[8];
        if (ti.tm_year >= (2020 - 1900)) {
            strftime(buf, sizeof(buf), "%H:%M", &ti);
        } else {
            snprintf(buf, sizeof(buf), "--:--");
        }
        lv_label_set_text(s_clock_lbl, buf);
    }
    if (s_wifi_icon) {
        if (wifi_manager_is_connected()) lv_obj_clear_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);
        else                             lv_obj_add_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_bt_icon) {
        bt_state_t bt = bt_manager_state();
        bool on = (bt == BT_STATE_ADVERTISING || bt == BT_STATE_CONNECTED);
        if (on) lv_obj_clear_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_add_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

static void idle_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_current_tool || !s_menu_roller || ui_watchface_active()) return;
    update_statusbar();
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
        int written = lv_snprintf(s_menu_options + used, MENU_OPTIONS_BUF_SIZE - used,
                                  "%s  %s%s", icon, name, sep);
        if (written <= 0 || (size_t)written >= (MENU_OPTIONS_BUF_SIZE - used)) break;
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
    s_pos_arc = NULL;
    s_nav_zone = NULL;
    s_clock_lbl = s_wifi_icon = s_bt_icon = NULL;
    s_current_tool = tool;

    if (tool->open) tool->open(lv_scr_act());
}

/* Recolorea el acento al cambiar de selección con el scroll. */
static void roller_value_changed(lv_event_t *e)
{
    (void)e;
    if (!s_menu_roller) return;
    apply_accent(lv_roller_get_selected(s_menu_roller));
}

/*
 * Abrir tocando el ícono: solo si el toque cae sobre la fila central y en la
 * zona izquierda (donde está el emoji/símbolo). Un tap en el nombre o fuera del
 * centro solo desplaza/selecciona, sin abrir.
 */
static void roller_click_cb(lv_event_t *e)
{
    (void)e;
    if (!s_menu_roller) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    lv_area_t a;
    lv_obj_get_coords(s_menu_roller, &a);
    lv_coord_t cy = (a.y1 + a.y2) / 2;

    /* Solo la fila CENTRAL (la resaltada): banda ceñida a su alto, para no
       disparar con los íconos de las filas de arriba/abajo. */
    if (p.y < cy - CENTER_BAND || p.y > cy + CENTER_BAND) return;
    /* Solo la zona del ícono centrado (no el nombre ni el costado izquierdo). */
    if (p.x < a.x1 + ICON_ZONE_X0 || p.x > a.x1 + ICON_ZONE_X1) return;

    uint16_t selected = lv_roller_get_selected(s_menu_roller);
    if (selected >= (uint16_t)g_tools_count) return;
    s_return_pos = selected;
    do_open_tool((int)selected);
}

/*
 * Navegación rápida: tocar/arrastrar sobre la barra inferior salta a esa
 * posición de la lista (mapea x -> índice). El roller sigue de inmediato.
 */
static void nav_scrub_cb(lv_event_t *e)
{
    (void)e;
    if (!s_menu_roller || !s_nav_zone || g_tools_count <= 0) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    lv_area_t a;
    lv_obj_get_coords(s_nav_zone, &a);
    lv_coord_t w = lv_area_get_width(&a);
    if (w <= 0) return;

    int rel = p.x - a.x1;
    if (rel < 0) rel = 0;
    if (rel > w) rel = w;

    int idx = (rel * g_tools_count) / (w + 1);
    if (idx >= g_tools_count) idx = g_tools_count - 1;
    if (idx < 0) idx = 0;

    if ((uint16_t)idx != lv_roller_get_selected(s_menu_roller)) {
        lv_roller_set_selected(s_menu_roller, (uint16_t)idx, LV_ANIM_OFF);
        apply_accent((uint16_t)idx);
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

/* Arco decorativo fino pegado al borde circular. */
static void create_frame_arc(lv_obj_t *parent)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, 236, 236);
    lv_obj_center(arc);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_value(arc, 0);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(arc, 3, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x142433), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
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

    /* Marco decorativo (detrás de todo) */
    create_frame_arc(lv_scr_act());

    /* Arco de posición (segmento inferior), teñido con el acento. Es el
       indicador visual; la interacción va por una zona táctil aparte. */
    s_pos_arc = lv_arc_create(lv_scr_act());
    lv_obj_set_size(s_pos_arc, 224, 224);
    lv_obj_center(s_pos_arc);
    lv_arc_set_bg_angles(s_pos_arc, 55, 125);          /* segmento inferior */
    lv_arc_set_mode(s_pos_arc, LV_ARC_MODE_REVERSE);   /* llena de izq->der con el índice */
    lv_arc_set_range(s_pos_arc, 0, (g_tools_count > 1) ? (g_tools_count - 1) : 1);
    lv_obj_clear_flag(s_pos_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_pos_arc, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_pos_arc, lv_color_hex(0x142433), LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_pos_arc, 5, LV_PART_INDICATOR);
    /* Perilla pequeña que marca la posición actual */
    lv_obj_set_style_bg_opa(s_pos_arc, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_radius(s_pos_arc, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_pos_arc, 3, LV_PART_KNOB);

    /* --- Barra de estado superior: reloj + Wi-Fi/BT --- */
    s_clock_lbl = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(s_clock_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_clock_lbl, lv_color_hex(0x8899AA), 0);
    lv_label_set_text(s_clock_lbl, "--:--");
    lv_obj_align(s_clock_lbl, LV_ALIGN_TOP_MID, 0, 20);

    s_wifi_icon = lv_label_create(lv_scr_act());
    lv_label_set_text(s_wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_wifi_icon, lv_color_hex(0x8899AA), 0);
    lv_obj_align(s_wifi_icon, LV_ALIGN_TOP_MID, -40, 21);
    lv_obj_add_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);

    s_bt_icon = lv_label_create(lv_scr_act());
    lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(s_bt_icon, lv_color_hex(0x8899AA), 0);
    lv_obj_align(s_bt_icon, LV_ALIGN_TOP_MID, 40, 21);
    lv_obj_add_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);

    /* --- Roller central --- */
    s_menu_roller = lv_roller_create(lv_scr_act());
    lv_roller_set_options(s_menu_roller, s_menu_options, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(s_menu_roller, 3);
    lv_obj_set_width(s_menu_roller, 210);
    lv_obj_align(s_menu_roller, LV_ALIGN_CENTER, 0, -2);
    lv_obj_add_event_cb(s_menu_roller, roller_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_menu_roller, roller_click_cb, LV_EVENT_CLICKED, NULL);

    /* Animación de asentado suave al soltar el scroll */
    lv_obj_set_style_anim_time(s_menu_roller, 280, LV_PART_MAIN);

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

    /* Zona táctil invisible sobre la barra inferior: navegación rápida.
       Tocar salta a esa posición; arrastrar hace scrub por la lista. */
    s_nav_zone = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_nav_zone);
    lv_obj_set_size(s_nav_zone, 180, 48);
    lv_obj_align(s_nav_zone, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_add_flag(s_nav_zone, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_nav_zone, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_nav_zone, nav_scrub_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_nav_zone, nav_scrub_cb, LV_EVENT_PRESSING, NULL);

    /* Volver a la posición desde la que se abrió la última tool (default 0) */
    if (s_return_pos < (uint16_t)g_tools_count) {
        lv_roller_set_selected(s_menu_roller, s_return_pos, LV_ANIM_OFF);
    }

    apply_accent(lv_roller_get_selected(s_menu_roller));
    update_statusbar();
}

void ui_menu_show(void)
{
    create_main_menu();
}
