/*
 * ui_menu.c — Menú principal para pantalla redonda.
 *
 * Lista de tarjetas con el mismo lenguaje visual que la tool Config: cada
 * herramienta es una píldora con su ícono en un chip del color de la propia
 * tool. La lista se CURVA siguiendo el borde circular y se atenúa hacia los
 * extremos, y hace snap al centro.
 *
 * Antes era un roller de texto donde había que tocar exactamente el ícono para
 * abrir, porque un toque en cualquier otro lado abría sin querer al desplazar.
 * Con una lista normal eso no hace falta: LVGL no emite el click si el dedo
 * termina desplazando, así que toda la tarjeta es pulsable.
 *
 * Arriba, una barra de estado (reloj + Wi-Fi/BT) sobre un fondo propio para
 * que se lea aunque pase una tarjeta por debajo.
 */
#include "ui_menu.h"

#include <stdio.h>
#include <time.h>

#include "esp_log.h"
#include "tool.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "ui_watchface.h"
#include "ui_theme.h"

#include <math.h>

static const char *TAG = "menu";

#define IDLE_TIMEOUT_MS 20000
/* Geometría de la lista curvada (misma que usa Config). */
#define SCREEN_R  120
#define CARD_W    178
#define CARD_H     54
#define LIST_PAD  ((240 - CARD_H) / 2)

static lv_obj_t *s_list = NULL;
static lv_obj_t *s_cards[64];
static lv_obj_t *s_clock_lbl = NULL;
static lv_obj_t *s_wifi_icon = NULL;
static lv_obj_t *s_bt_icon = NULL;
static const tool_t *s_current_tool = NULL;
static lv_timer_t *s_idle_timer = NULL;

/* Mapa fila del roller -> índice en g_tools[]. Las tools marcadas 'hidden' no
 * salen en el menú (viven dentro de Config), así que los índices ya no
 * coinciden y hay que traducirlos. */
static int s_visible[64];
static int s_visible_count = 0;

/* Tool a la que vuelve el gesto de "atrás" (NULL = menú principal). La fija
 * ui_menu_open_tool para que salir de Wi-Fi devuelva a Config, no al menú. */
static const tool_t *s_return_tool = NULL;

/* Posición del menú a restaurar al volver de una tool (default 0 al encender) */
static uint16_t s_return_pos = 0;

#define ACCENT_DEFAULT 0x2E82C8

/* Tool de la fila 'pos' del roller (traduce por el mapa de visibles). */
static const tool_t *tool_at(int pos)
{
    if (pos < 0 || pos >= s_visible_count) return NULL;
    return g_tools[s_visible[pos]];
}

static uint32_t pos_accent(int pos)
{
    const tool_t *t = tool_at(pos);
    return (t && t->accent) ? t->accent : ACCENT_DEFAULT;
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
    if (s_current_tool || !s_list || ui_watchface_active()) return;
    update_statusbar();
    if (lv_disp_get_inactive_time(NULL) > IDLE_TIMEOUT_MS) {
        ui_watchface_show();
    }
}

/* Qué herramientas salen en el menú (las 'hidden' viven dentro de Config). */
static void build_visible(void)
{
    s_visible_count = 0;
    for (int i = 0; i < g_tools_count &&
         s_visible_count < (int)(sizeof(s_visible) / sizeof(s_visible[0])); i++) {
        if (g_tools[i] && g_tools[i]->hidden) continue;
        s_visible[s_visible_count++] = i;
    }
}

static void open_tool_ptr(const tool_t *tool)
{
    if (!tool) return;

    ESP_LOGI(TAG, "Abriendo herramienta: %s", tool->name ? tool->name : "(sin nombre)");

    lv_obj_clean(lv_scr_act());
    s_list = NULL;
    s_clock_lbl = s_wifi_icon = s_bt_icon = NULL;
    s_current_tool = tool;

    if (tool->open) tool->open(lv_scr_act());
}

/* Apertura diferida: se llama desde el callback de un botón de otra tool, así
 * que no se puede borrar su UI mientras LVGL procesa el evento. */
static void open_async_cb(void *param)
{
    close_current_tool();
    lv_obj_clean(lv_scr_act());
    open_tool_ptr((const tool_t *)param);
}

void ui_menu_open_tool(const tool_t *tool)
{
    if (!tool) return;
    s_return_tool = s_current_tool;   /* volver a quien la abrió */
    lv_async_call(open_async_cb, (void *)tool);
}

static void screen_gesture_cb(lv_event_t *e)
{
    (void)e;
    if (!s_current_tool) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;

    if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
        /* Si esta tool se abrió desde otra (Config), volver a esa. */
        const tool_t *back = s_return_tool;
        s_return_tool = NULL;
        close_current_tool();
        lv_obj_clean(lv_scr_act());
        if (back) open_tool_ptr(back);
        else      create_main_menu();
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

/*
 * Curvado: cada tarjeta se corre en X siguiendo la circunferencia y se atenúa
 * al alejarse del centro. Es lo que hace que la lista abrace el borde redondo
 * en vez de quedar cortada en las esquinas.
 */
static void list_curve_cb(lv_event_t *e)
{
    lv_obj_t *cont = lv_event_get_target(e);
    lv_area_t ca;
    lv_obj_get_coords(cont, &ca);
    lv_coord_t center_y = (ca.y1 + ca.y2) / 2;

    uint32_t n = lv_obj_get_child_cnt(cont);
    for (uint32_t i = 0; i < n; i++) {
        lv_obj_t *child = lv_obj_get_child(cont, i);
        lv_area_t a;
        lv_obj_get_coords(child, &a);

        int dy = ((a.y1 + a.y2) / 2) - center_y;
        if (dy < 0) dy = -dy;
        if (dy > SCREEN_R) dy = SCREEN_R;

        int inset = SCREEN_R - (int)sqrtf((float)(SCREEN_R * SCREEN_R - dy * dy));
        lv_obj_set_style_translate_x(child, inset / 2, 0);

        int opa = 255 - (dy * 170) / SCREEN_R;
        if (opa < 60) opa = 60;
        lv_obj_set_style_opa(child, (lv_opa_t)opa, 0);
    }
}

static void card_click_cb(lv_event_t *e)
{
    int pos = (int)(intptr_t)lv_event_get_user_data(e);
    const tool_t *t = tool_at(pos);
    if (!t) return;
    s_return_pos = (uint16_t)pos;
    s_return_tool = NULL;              /* se abrió desde el menú */
    open_tool_ptr(t);
}

/* Tarjeta: chip circular con el color de la tool + su nombre. */
static lv_obj_t *card_create(int pos)
{
    const tool_t *t = tool_at(pos);

    lv_obj_t *card = lv_obj_create(s_list);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, CARD_W, CARD_H);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, CARD_H / 2, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD_PRESS), LV_STATE_PRESSED);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, card_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)pos);

    lv_obj_t *chip = lv_obj_create(card);
    lv_obj_remove_style_all(chip);
    lv_obj_set_size(chip, 38, 38);
    lv_obj_align(chip, LV_ALIGN_LEFT_MID, 8, 0);
    lv_obj_set_style_bg_color(chip, lv_color_hex(pos_accent(pos)), 0);
    lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(chip, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ic = lv_label_create(chip);
    lv_label_set_text(ic, (t && t->icon) ? t->icon : LV_SYMBOL_DUMMY);
    lv_obj_set_style_text_color(ic, lv_color_hex(UI_BG), 0);
    lv_obj_center(ic);

    lv_obj_t *name = lv_label_create(card);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(name, lv_color_hex(UI_TEXT), 0);
    lv_label_set_text(name, (t && t->name) ? t->name : "Tool");
    lv_obj_align(name, LV_ALIGN_LEFT_MID, 56, 0);

    return card;
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

    build_visible();

    /* Marco decorativo (detrás de todo) */
    create_frame_arc(lv_scr_act());

    /* --- Lista curvada de herramientas --- */
    s_list = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_list);
    lv_obj_set_size(s_list, 240, 240);
    lv_obj_center(s_list);
    lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_list, 8, 0);
    /* Relleno para que la primera y la última puedan quedar centradas. */
    lv_obj_set_style_pad_top(s_list, LIST_PAD, 0);
    lv_obj_set_style_pad_bottom(s_list, LIST_PAD, 0);
    lv_obj_set_scroll_dir(s_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_snap_y(s_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_add_event_cb(s_list, list_curve_cb, LV_EVENT_SCROLL, NULL);

    for (int i = 0; i < s_visible_count; i++) {
        s_cards[i] = card_create(i);
    }

    /* --- Barra de estado: reloj + Wi-Fi/BT, con fondo propio ---
       Va después de la lista para quedar por encima, y con un fondo suave para
       que se lea aunque pase una tarjeta por debajo. */
    lv_obj_t *bar = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, 130, 26);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 12);
    lv_obj_set_style_bg_color(bar, lv_color_hex(UI_SCREEN), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_80, 0);
    lv_obj_set_style_radius(bar, 13, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);

    s_clock_lbl = lv_label_create(bar);
    lv_obj_set_style_text_font(s_clock_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_clock_lbl, lv_color_hex(UI_TITLE), 0);
    lv_label_set_text(s_clock_lbl, "--:--");
    lv_obj_center(s_clock_lbl);

    s_wifi_icon = lv_label_create(bar);
    lv_label_set_text(s_wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_wifi_icon, lv_color_hex(UI_TITLE), 0);
    lv_obj_align(s_wifi_icon, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_add_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);

    s_bt_icon = lv_label_create(bar);
    lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(s_bt_icon, lv_color_hex(UI_TITLE), 0);
    lv_obj_align(s_bt_icon, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_add_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);

    /* Volver a donde estaba al abrir la última tool. */
    lv_obj_update_layout(s_list);
    if (s_return_pos < (uint16_t)s_visible_count && s_cards[s_return_pos]) {
        lv_obj_scroll_to_view(s_cards[s_return_pos], LV_ANIM_OFF);
    }
    lv_event_send(s_list, LV_EVENT_SCROLL, NULL);   /* curvar ya */

    update_statusbar();
}

void ui_menu_show(void)
{
    create_main_menu();
}
