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
 * Contenido, en una lista desplazable:
 *   - Brillo (slider, se aplica en vivo)
 *   - Apagar pantalla (ciclo de tiempos, "nunca" incluido)
 *   - Despertar al mover (on/off) y su sensibilidad
 *   - Accesos a Wi-Fi, Redes, Info de red, QR WiFi y Bluetooth
 */
#include "tool.h"
#include "ui_menu.h"
#include "ui_power.h"

#include <stdio.h>

/* Tools que esta pantalla agrupa (definidas en sus propios .c). */
extern const tool_t tool_wifi;
extern const tool_t tool_wifiscan;
extern const tool_t tool_netinfo;
extern const tool_t tool_wifiqr;
extern const tool_t tool_bt;

/* Tiempos ofrecidos para apagar la pantalla, en segundos (0 = nunca). */
static const int SLEEP_OPTS[] = { 15, 30, 45, 60, 120, 300, 0 };
#define SLEEP_OPTS_N ((int)(sizeof(SLEEP_OPTS) / sizeof(SLEEP_OPTS[0])))

static lv_obj_t *s_sleep_lbl = NULL;
static lv_obj_t *s_motion_lbl = NULL;
static lv_obj_t *s_sens_lbl = NULL;
static lv_obj_t *s_bright_lbl = NULL;

/* ------------------------------- Etiquetas ------------------------------- */

static void refresh_sleep_lbl(void)
{
    if (!s_sleep_lbl) return;
    int s = ui_power_get_sleep_s();
    char b[32];
    if (s == 0)        snprintf(b, sizeof(b), "Apagar: nunca");
    else if (s < 60)   snprintf(b, sizeof(b), "Apagar: %d s", s);
    else               snprintf(b, sizeof(b), "Apagar: %d min", s / 60);
    lv_label_set_text(s_sleep_lbl, b);
}

static void refresh_motion_lbl(void)
{
    if (s_motion_lbl) {
        lv_label_set_text_fmt(s_motion_lbl, "Despertar al mover: %s",
                              ui_power_get_motion_wake() ? "si" : "no");
    }
    if (s_sens_lbl) {
        lv_label_set_text_fmt(s_sens_lbl, "Sensibilidad: %s",
                              ui_power_sens_name(ui_power_get_sensitivity()));
        /* Sin despertar por movimiento, la sensibilidad no aplica. */
        lv_obj_set_style_text_color(s_sens_lbl,
            ui_power_get_motion_wake() ? lv_color_hex(0xDDE6F0) : lv_color_hex(0x5A6B7A), 0);
    }
}

static void refresh_bright_lbl(void)
{
    if (s_bright_lbl) lv_label_set_text_fmt(s_bright_lbl, "Brillo  %d %%", ui_power_get_brightness());
}

/* -------------------------------- Callbacks ------------------------------ */

static void bright_cb(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target(e);
    ui_power_set_brightness((int)lv_slider_get_value(sl));
    refresh_bright_lbl();
}

static void sleep_cb(lv_event_t *e)
{
    (void)e;
    int cur = ui_power_get_sleep_s();
    int idx = 0;
    for (int i = 0; i < SLEEP_OPTS_N; i++) if (SLEEP_OPTS[i] == cur) { idx = i; break; }
    ui_power_set_sleep_s(SLEEP_OPTS[(idx + 1) % SLEEP_OPTS_N]);
    refresh_sleep_lbl();
}

static void motion_cb(lv_event_t *e)
{
    (void)e;
    ui_power_set_motion_wake(!ui_power_get_motion_wake());
    refresh_motion_lbl();
}

static void sens_cb(lv_event_t *e)
{
    (void)e;
    int s = (int)ui_power_get_sensitivity() + 1;
    if (s >= UI_POWER_SENS_COUNT) s = 0;
    ui_power_set_sensitivity((ui_power_sens_t)s);
    refresh_motion_lbl();
}

static void open_tool_cb(lv_event_t *e)
{
    const tool_t *t = (const tool_t *)lv_event_get_user_data(e);
    ui_menu_open_tool(t);
}

/* ------------------------------ Constructores ---------------------------- */

/* Fila pulsable con texto; devuelve su label para poder refrescarlo. */
static lv_obj_t *add_row(lv_obj_t *list, const char *icon, const char *text,
                         lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *btn = lv_list_add_btn(list, icon, text);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x22303F), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(btn, &lv_font_montserrat_14, 0);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    /* El label del botón es su último hijo (después del ícono). */
    lv_obj_t *lbl = lv_obj_get_child(btn, -1);
    return lbl;
}

static void add_separator(lv_obj_t *list, const char *text)
{
    lv_obj_t *lbl = lv_label_create(list);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x7F8C8D), 0);
    lv_obj_set_style_pad_top(lbl, 6, 0);
    lv_label_set_text(lbl, text);
}

static void settings_open(lv_obj_t *parent)
{
    lv_obj_t *title = lv_label_create(parent);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(title, "Config");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    lv_obj_t *list = lv_list_create(parent);
    lv_obj_set_size(list, 200, 176);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 12);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_row(list, 6, 0);

    /* --- Pantalla --- */
    add_separator(list, "Pantalla");

    /* El brillo va en su propio contenedor: label arriba, slider abajo. */
    lv_obj_t *bcont = lv_obj_create(list);
    lv_obj_remove_style_all(bcont);
    lv_obj_set_size(bcont, LV_PCT(100), 54);
    lv_obj_clear_flag(bcont, LV_OBJ_FLAG_SCROLLABLE);

    s_bright_lbl = lv_label_create(bcont);
    lv_obj_set_style_text_font(s_bright_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_bright_lbl, lv_color_hex(0xDDE6F0), 0);
    lv_obj_align(s_bright_lbl, LV_ALIGN_TOP_LEFT, 4, 0);
    refresh_bright_lbl();

    lv_obj_t *slider = lv_slider_create(bcont);
    lv_obj_set_size(slider, 168, 12);
    lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_slider_set_range(slider, 10, 100);
    lv_slider_set_value(slider, ui_power_get_brightness(), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x2A3A48), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x35D07F), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_white(), LV_PART_KNOB);
    /* VALUE_CHANGED se dispara mientras se arrastra: el brillo acompaña al dedo. */
    lv_obj_add_event_cb(slider, bright_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_sleep_lbl  = add_row(list, LV_SYMBOL_POWER, "", sleep_cb, NULL);
    refresh_sleep_lbl();

    s_motion_lbl = add_row(list, LV_SYMBOL_REFRESH, "", motion_cb, NULL);
    s_sens_lbl   = add_row(list, LV_SYMBOL_SETTINGS, "", sens_cb, NULL);
    refresh_motion_lbl();

    /* --- Comunicaciones --- */
    add_separator(list, "Comunicaciones");
    add_row(list, LV_SYMBOL_WIFI,     "Wi-Fi",       open_tool_cb, (void *)&tool_wifi);
    add_row(list, LV_SYMBOL_LIST,     "Redes",       open_tool_cb, (void *)&tool_wifiscan);
    add_row(list, LV_SYMBOL_DOWNLOAD, "Info de red", open_tool_cb, (void *)&tool_netinfo);
    add_row(list, LV_SYMBOL_IMAGE,    "QR WiFi",     open_tool_cb, (void *)&tool_wifiqr);
    add_row(list, LV_SYMBOL_BLUETOOTH,"Bluetooth",   open_tool_cb, (void *)&tool_bt);
}

static void settings_close(void)
{
    s_sleep_lbl = s_motion_lbl = s_sens_lbl = s_bright_lbl = NULL;
}

const tool_t tool_settings = {
    .name = "Config",
    .icon = LV_SYMBOL_SETTINGS,
    .accent = 0x95A5A6,
    .open = settings_open,
    .close = settings_close,
};
