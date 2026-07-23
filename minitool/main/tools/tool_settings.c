/*
 * tool_settings.c — Configuración general del reloj.
 *
 * Junta en un solo lugar lo que antes estaba desperdigado por el menú: los
 * parámetros del sistema (pantalla) y las herramientas de comunicaciones
 * (Wi-Fi, Bluetooth, red). Esas tools siguen existiendo tal cual, pero se
 * marcan 'hidden' en su definición: desaparecen del menú principal y se abren
 * desde acá con ui_menu_open_tool(). El gesto de volver las devuelve a esta
 * pantalla, no al menú.
 *
 * La interfaz sigue las convenciones de un smartwatch actual:
 *
 *   - Filas tipo "pill" con el ícono en un chip circular de color, el nombre
 *     del ajuste y su valor debajo en gris. Objetivos táctiles grandes.
 *   - Lista CURVADA: cada fila se corre horizontalmente siguiendo el borde
 *     circular y se atenúa al alejarse del centro. En una pantalla redonda una
 *     lista recta desperdicia las esquinas y se corta fea.
 *   - Snap al centro: el scroll siempre deja una fila centrada, que es la que
 *     está "en foco".
 *   - Los booleanos son switches de verdad, no texto que dice si/no.
 *   - Los valores con varias opciones abren una SUBPÁGINA con la lista y un
 *     check en el elegido, en vez de ciclar a ciegas con cada toque. El brillo
 *     abre un arco a pantalla completa, que es el control natural de un
 *     display redondo.
 */
#include "tool.h"
#include "ui_menu.h"
#include "ui_power.h"

#include <math.h>
#include <stdio.h>

/* Tools que esta pantalla agrupa (definidas en sus propios .c). */
extern const tool_t tool_wifi;
extern const tool_t tool_wifiscan;
extern const tool_t tool_netinfo;
extern const tool_t tool_wifiqr;
extern const tool_t tool_bt;

/* Geometría */
#define SCREEN_R     120    /* radio de la pantalla, para curvar la lista */
#define CARD_W       178
#define CARD_H        54
#define LIST_PAD     ((240 - CARD_H) / 2)   /* deja centrar la 1ª y la última */

/* Paleta */
#define COL_CARD     0x1A2733
#define COL_TITLE    0xDDE6F0
#define COL_VALUE    0x7F8C8D
#define COL_HEADER   0x5A6B7A

/* Tiempos ofrecidos para apagar la pantalla, en segundos (0 = nunca). */
static const int SLEEP_OPTS[] = { 15, 30, 45, 60, 120, 300, 0 };
#define SLEEP_OPTS_N ((int)(sizeof(SLEEP_OPTS) / sizeof(SLEEP_OPTS[0])))

static lv_obj_t *s_list = NULL;
static lv_obj_t *s_page = NULL;      /* subpágina abierta (overlay) */
static lv_obj_t *s_val_bright = NULL;
static lv_obj_t *s_val_sleep = NULL;
static lv_obj_t *s_val_sens = NULL;
static lv_obj_t *s_motion_sw = NULL;

/* ---------------------------- Textos de valor ---------------------------- */

static void fmt_sleep(int s, char *out, int n)
{
    if (s == 0)      snprintf(out, n, "Nunca");
    else if (s < 60) snprintf(out, n, "%d segundos", s);
    else             snprintf(out, n, "%d minuto%s", s / 60, (s / 60) == 1 ? "" : "s");
}

static void refresh_values(void)
{
    char b[24];
    if (s_val_bright) lv_label_set_text_fmt(s_val_bright, "%d %%", ui_power_get_brightness());
    if (s_val_sleep)  { fmt_sleep(ui_power_get_sleep_s(), b, sizeof(b)); lv_label_set_text(s_val_sleep, b); }
    if (s_val_sens)   lv_label_set_text(s_val_sens, ui_power_sens_name(ui_power_get_sensitivity()));

    if (s_val_sens) {
        /* Sin despertar por movimiento, la sensibilidad no aplica. */
        bool on = ui_power_get_motion_wake();
        lv_obj_set_style_text_color(s_val_sens, lv_color_hex(on ? COL_VALUE : 0x3A4A58), 0);
    }
}

/* ------------------------------ Lista curvada ---------------------------- */

/*
 * Curvado: cada fila se corre en X siguiendo la circunferencia (x = R - √(R²-dy²))
 * y se atenúa según se aleja del centro vertical. Es lo que hace que la lista
 * "abrace" el borde redondo en vez de quedar cortada en las esquinas.
 */
static void list_scroll_cb(lv_event_t *e)
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
        lv_coord_t child_y = (a.y1 + a.y2) / 2;

        int dy = child_y - center_y;
        if (dy < 0) dy = -dy;
        if (dy > SCREEN_R) dy = SCREEN_R;

        /* Sangría siguiendo el círculo, a media escala para que no exagere. */
        int inset = SCREEN_R - (int)sqrtf((float)(SCREEN_R * SCREEN_R - dy * dy));
        lv_obj_set_style_translate_x(child, inset / 2, 0);

        /* Lo que se aleja del foco se apaga. */
        int opa = 255 - (dy * 170) / SCREEN_R;
        if (opa < 60) opa = 60;
        lv_obj_set_style_opa(child, (lv_opa_t)opa, 0);
    }
}

/* ------------------------------ Subpáginas ------------------------------- */

static void page_close(void)
{
    if (s_page) { lv_obj_del(s_page); s_page = NULL; }
    refresh_values();
}

static void page_back_cb(lv_event_t *e) { (void)e; page_close(); }

/* Overlay a pantalla completa con título y chevron de volver arriba a la
 * izquierda, como en watchOS. Devuelve el contenedor para llenarlo. */
static lv_obj_t *page_create(const char *title)
{
    if (s_page) page_close();

    s_page = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_page);
    lv_obj_set_size(s_page, 240, 240);
    lv_obj_center(s_page);
    lv_obj_set_style_bg_color(s_page, lv_color_hex(0x0A0E12), 0);
    lv_obj_set_style_bg_opa(s_page, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_page, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back = lv_btn_create(s_page);
    lv_obj_set_size(back, 38, 30);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 30, 16);
    lv_obj_set_style_radius(back, 15, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x22303F), 0);
    lv_obj_add_event_cb(back, page_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_center(bl);

    lv_obj_t *t = lv_label_create(s_page);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(t, title);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 14, 20);

    return s_page;
}

/* --- Subpágina: brillo (arco, el control natural de una pantalla redonda) - */

static lv_obj_t *s_bright_arc_lbl = NULL;

static void bright_arc_cb(lv_event_t *e)
{
    lv_obj_t *arc = lv_event_get_target(e);
    int v = (int)lv_arc_get_value(arc);
    ui_power_set_brightness(v);
    if (s_bright_arc_lbl) lv_label_set_text_fmt(s_bright_arc_lbl, "%d%%", v);
}

static void page_brightness(lv_event_t *e)
{
    (void)e;
    lv_obj_t *p = page_create("Brillo");

    lv_obj_t *arc = lv_arc_create(p);
    lv_obj_set_size(arc, 176, 176);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 12);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_range(arc, 10, 100);
    lv_arc_set_value(arc, ui_power_get_brightness());
    lv_obj_set_style_arc_width(arc, 12, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x22303F), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 12, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0xF1C40F), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(arc, lv_color_white(), LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 6, LV_PART_KNOB);
    /* VALUE_CHANGED se emite mientras se arrastra: el brillo sigue al dedo. */
    lv_obj_add_event_cb(arc, bright_arc_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_bright_arc_lbl = lv_label_create(p);
    lv_obj_set_style_text_font(s_bright_arc_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_bright_arc_lbl, lv_color_white(), 0);
    lv_label_set_text_fmt(s_bright_arc_lbl, "%d%%", ui_power_get_brightness());
    lv_obj_align(s_bright_arc_lbl, LV_ALIGN_CENTER, 0, 8);
}

/* --- Subpáginas de opciones (una lista con check en la elegida) ----------- */

typedef void (*opt_apply_t)(int index);

static opt_apply_t s_opt_apply = NULL;

static void opt_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_opt_apply) s_opt_apply(idx);
    page_close();
}

static void page_options(const char *title, const char **labels, int n,
                         int selected, opt_apply_t apply)
{
    lv_obj_t *p = page_create(title);
    s_opt_apply = apply;

    lv_obj_t *list = lv_obj_create(p);
    lv_obj_remove_style_all(list);
    lv_obj_set_size(list, 200, 172);
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 6, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);

    for (int i = 0; i < n; i++) {
        lv_obj_t *btn = lv_btn_create(list);
        lv_obj_set_size(btn, LV_PCT(100), 40);
        lv_obj_set_style_radius(btn, 20, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(i == selected ? 0x2A4356 : COL_CARD), 0);
        lv_obj_add_event_cb(btn, opt_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(COL_TITLE), 0);
        lv_label_set_text(lbl, labels[i]);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 14, 0);

        if (i == selected) {
            lv_obj_t *ok = lv_label_create(btn);
            lv_label_set_text(ok, LV_SYMBOL_OK);
            lv_obj_set_style_text_color(ok, lv_color_hex(0x35D07F), 0);
            lv_obj_align(ok, LV_ALIGN_RIGHT_MID, -12, 0);
        }
    }
}

static void apply_sleep(int idx)
{
    if (idx >= 0 && idx < SLEEP_OPTS_N) ui_power_set_sleep_s(SLEEP_OPTS[idx]);
}

static void page_sleep(lv_event_t *e)
{
    (void)e;
    static char bufs[SLEEP_OPTS_N][24];
    const char *labels[SLEEP_OPTS_N];
    int sel = 0;
    for (int i = 0; i < SLEEP_OPTS_N; i++) {
        fmt_sleep(SLEEP_OPTS[i], bufs[i], sizeof(bufs[i]));
        labels[i] = bufs[i];
        if (SLEEP_OPTS[i] == ui_power_get_sleep_s()) sel = i;
    }
    page_options("Apagar pantalla", labels, SLEEP_OPTS_N, sel, apply_sleep);
}

static void apply_sens(int idx) { ui_power_set_sensitivity((ui_power_sens_t)idx); }

static void page_sens(lv_event_t *e)
{
    (void)e;
    const char *labels[UI_POWER_SENS_COUNT];
    for (int i = 0; i < UI_POWER_SENS_COUNT; i++) labels[i] = ui_power_sens_name((ui_power_sens_t)i);
    page_options("Sensibilidad", labels, UI_POWER_SENS_COUNT,
                 (int)ui_power_get_sensitivity(), apply_sens);
}

/* -------------------------------- Callbacks ------------------------------ */

static void motion_sw_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    ui_power_set_motion_wake(lv_obj_has_state(sw, LV_STATE_CHECKED));
    refresh_values();
}

static void open_tool_cb(lv_event_t *e)
{
    ui_menu_open_tool((const tool_t *)lv_event_get_user_data(e));
}

/* ------------------------------ Construcción ----------------------------- */

/* Fila: chip circular con el ícono, título y (opcional) valor debajo. */
static lv_obj_t *card_create(const char *icon, uint32_t chip_color,
                             const char *title, bool with_value,
                             lv_event_cb_t cb, void *ud)
{
    lv_obj_t *card = lv_obj_create(s_list);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, CARD_W, CARD_H);
    lv_obj_set_style_bg_color(card, lv_color_hex(COL_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, CARD_H / 2, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    if (cb) {
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(card, cb, LV_EVENT_CLICKED, ud);
        /* Realimentación al tocar, como cualquier lista táctil moderna. */
        lv_obj_set_style_bg_color(card, lv_color_hex(0x27384A), LV_STATE_PRESSED);
    }

    lv_obj_t *chip = lv_obj_create(card);
    lv_obj_remove_style_all(chip);
    lv_obj_set_size(chip, 36, 36);
    lv_obj_align(chip, LV_ALIGN_LEFT_MID, 8, 0);
    lv_obj_set_style_bg_color(chip, lv_color_hex(chip_color), 0);
    lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(chip, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ic = lv_label_create(chip);
    lv_label_set_text(ic, icon);
    lv_obj_set_style_text_color(ic, lv_color_hex(0x0A0E12), 0);
    lv_obj_center(ic);

    lv_obj_t *t = lv_label_create(card);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(COL_TITLE), 0);
    lv_label_set_text(t, title);
    lv_obj_align(t, with_value ? LV_ALIGN_LEFT_MID : LV_ALIGN_LEFT_MID, 52, with_value ? -9 : 0);

    return card;
}

/* Añade el label de valor a una fila creada con with_value. */
static lv_obj_t *card_value(lv_obj_t *card)
{
    lv_obj_t *v = lv_label_create(card);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(v, lv_color_hex(COL_VALUE), 0);
    lv_label_set_text(v, "");
    lv_obj_align(v, LV_ALIGN_LEFT_MID, 52, 10);
    return v;
}

static void header_create(const char *text)
{
    lv_obj_t *h = lv_label_create(s_list);
    lv_obj_set_style_text_font(h, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(h, lv_color_hex(COL_HEADER), 0);
    lv_obj_set_style_pad_left(h, 18, 0);
    lv_obj_set_style_pad_top(h, 8, 0);
    lv_label_set_text(h, text);
}

static void settings_open(lv_obj_t *parent)
{
    s_list = lv_obj_create(parent);
    lv_obj_remove_style_all(s_list);
    lv_obj_set_size(s_list, 240, 240);
    lv_obj_center(s_list);
    lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_list, 8, 0);
    /* Relleno arriba y abajo para que la primera y la última fila puedan
     * quedar centradas, como en cualquier lista de reloj. */
    lv_obj_set_style_pad_top(s_list, LIST_PAD, 0);
    lv_obj_set_style_pad_bottom(s_list, LIST_PAD, 0);
    lv_obj_set_scroll_dir(s_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_snap_y(s_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_add_event_cb(s_list, list_scroll_cb, LV_EVENT_SCROLL, NULL);

    header_create("Pantalla");

    lv_obj_t *c = card_create(LV_SYMBOL_EYE_OPEN, 0xF1C40F, "Brillo", true, page_brightness, NULL);
    s_val_bright = card_value(c);

    c = card_create(LV_SYMBOL_POWER, 0x3498DB, "Apagar", true, page_sleep, NULL);
    s_val_sleep = card_value(c);

    /* Fila con switch: el toggle vive en la fila, no abre subpágina. */
    c = card_create(LV_SYMBOL_REFRESH, 0x35D07F, "Al mover", false, NULL, NULL);
    s_motion_sw = lv_switch_create(c);
    lv_obj_set_size(s_motion_sw, 44, 24);
    lv_obj_align(s_motion_sw, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(s_motion_sw, lv_color_hex(0x35D07F), LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (ui_power_get_motion_wake()) lv_obj_add_state(s_motion_sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_motion_sw, motion_sw_cb, LV_EVENT_VALUE_CHANGED, NULL);

    c = card_create(LV_SYMBOL_SETTINGS, 0x9B59B6, "Sensibilidad", true, page_sens, NULL);
    s_val_sens = card_value(c);

    header_create("Comunicaciones");
    card_create(LV_SYMBOL_WIFI,      0x4AA8FF, "Wi-Fi",       false, open_tool_cb, (void *)&tool_wifi);
    card_create(LV_SYMBOL_EYE_OPEN,  0x2ED9A3, "Redes",       false, open_tool_cb, (void *)&tool_wifiscan);
    card_create(LV_SYMBOL_LIST,      0x8FA8C8, "Info de red", false, open_tool_cb, (void *)&tool_netinfo);
    card_create(LV_SYMBOL_IMAGE,     0x9B59B6, "QR WiFi",     false, open_tool_cb, (void *)&tool_wifiqr);
    card_create(LV_SYMBOL_BLUETOOTH, 0x3E7BFF, "Bluetooth",   false, open_tool_cb, (void *)&tool_bt);

    refresh_values();

    /* Curvar ya en el primer dibujado, sin esperar a que el usuario desplace. */
    lv_obj_update_layout(s_list);
    lv_event_send(s_list, LV_EVENT_SCROLL, NULL);
}

static void settings_close(void)
{
    page_close();
    s_list = NULL;
    s_val_bright = s_val_sleep = s_val_sens = s_motion_sw = NULL;
    s_bright_arc_lbl = NULL;
    s_opt_apply = NULL;
}

const tool_t tool_settings = {
    .name = "Config",
    .icon = LV_SYMBOL_SETTINGS,
    .accent = 0x95A5A6,
    .open = settings_open,
    .close = settings_close,
};
