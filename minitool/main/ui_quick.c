/*
 * ui_quick.c — Implementación del panel de ajustes rápidos.
 *
 * Vive en lv_layer_top, como la carátula y las notificaciones, para poder
 * aparecer encima de lo que sea que esté abierto.
 *
 * Tres botones redondos grandes arriba y el brillo abajo. Los botones son
 * conmutadores con estado visible: si están encendidos se pintan con su color;
 * apagados, en gris. En una pantalla de este tamaño el color dice más rápido
 * que cualquier texto.
 */
#include "ui_quick.h"
#include "ui_power.h"
#include "bt_manager.h"
#include "bsp.h"

#include "lvgl.h"
#include "esp_log.h"

static const char *TAG = "ui_quick";

static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_mute_btn = NULL;
static lv_obj_t *s_bt_btn = NULL;
static lv_obj_t *s_light_btn = NULL;
static lv_obj_t *s_bright_lbl = NULL;
static bool      s_light_on = false;

bool ui_quick_active(void) { return s_panel != NULL; }

void ui_quick_hide(void)
{
    if (!s_panel) return;
    /* Si la linterna quedó encendida desde acá, apagarla al cerrar: es un
     * atajo momentáneo, no un modo en el que uno se queda. */
    if (s_light_on) {
        s_light_on = false;
        bsp_backlight_set(ui_power_get_brightness());
        ui_power_inhibit(false);
    }
    lv_obj_del(s_panel);
    s_panel = NULL;
    s_mute_btn = s_bt_btn = s_light_btn = s_bright_lbl = NULL;
}

/* ------------------------------- Apariencia ------------------------------- */

static void paint(lv_obj_t *btn, bool on, uint32_t color_on)
{
    if (!btn) return;
    lv_obj_set_style_bg_color(btn, lv_color_hex(on ? color_on : 0x22303F), 0);
}

static void refresh(void)
{
    paint(s_mute_btn,  ui_power_get_mute(), 0xE0A030);
    paint(s_bt_btn,    bt_manager_state() != BT_STATE_OFF, 0x3E7BFF);
    paint(s_light_btn, s_light_on, 0xF1C40F);
    if (s_bright_lbl) lv_label_set_text_fmt(s_bright_lbl, "%d %%", ui_power_get_brightness());
}

/* -------------------------------- Acciones -------------------------------- */

static void mute_cb(lv_event_t *e)
{
    (void)e;
    ui_power_set_mute(!ui_power_get_mute());
    ESP_LOGI(TAG, "Silencio %s", ui_power_get_mute() ? "on" : "off");
    refresh();
}

static void bt_cb(lv_event_t *e)
{
    (void)e;
    if (bt_manager_state() == BT_STATE_OFF) bt_manager_start();
    else                                    bt_manager_stop();
    refresh();
}

/* Linterna momentánea: pantalla al máximo y sin dejar que se duerma. No abre
 * la tool, que además cicla colores; acá lo que se quiere es luz ya. */
static void light_cb(lv_event_t *e)
{
    (void)e;
    s_light_on = !s_light_on;
    if (s_light_on) {
        ui_power_inhibit(true);
        bsp_backlight_set(100);
    } else {
        bsp_backlight_set(ui_power_get_brightness());
        ui_power_inhibit(false);
    }
    refresh();
}

static void bright_cb(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target(e);
    ui_power_set_brightness((int)lv_slider_get_value(sl));
    if (s_bright_lbl) lv_label_set_text_fmt(s_bright_lbl, "%d %%", ui_power_get_brightness());
}

/* Deslizar hacia arriba cierra: es el gesto inverso al que lo abre. */
static void panel_gesture_cb(lv_event_t *e)
{
    (void)e;
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    if (lv_indev_get_gesture_dir(indev) == LV_DIR_TOP) ui_quick_hide();
}

/* ------------------------------ Construcción ------------------------------ */

static lv_obj_t *big_btn(lv_obj_t *parent, int x, const char *sym, lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, 56, 56);
    lv_obj_align(b, LV_ALIGN_CENTER, x, -22);
    lv_obj_set_style_radius(b, 28, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x22303F), 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(b, LV_OBJ_FLAG_EVENT_BUBBLE);   /* deja pasar el gesto */

    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0x0A0E12), 0);
    lv_label_set_text(l, sym);
    lv_obj_center(l);
    return b;
}

void ui_quick_show(void)
{
    if (s_panel) return;

    s_panel = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_panel);
    lv_obj_set_size(s_panel, 240, 240);
    lv_obj_center(s_panel);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(0x0A0E12), 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_panel, panel_gesture_cb, LV_EVENT_GESTURE, NULL);

    s_mute_btn  = big_btn(s_panel, -66, LV_SYMBOL_BELL,      mute_cb);
    s_light_btn = big_btn(s_panel,   0, LV_SYMBOL_CHARGE,    light_cb);
    s_bt_btn    = big_btn(s_panel,  66, LV_SYMBOL_BLUETOOTH, bt_cb);

    s_bright_lbl = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_bright_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_bright_lbl, lv_color_hex(0x8FA8C8), 0);
    lv_obj_align(s_bright_lbl, LV_ALIGN_CENTER, 0, 34);

    lv_obj_t *sl = lv_slider_create(s_panel);
    lv_obj_set_size(sl, 150, 14);
    lv_obj_align(sl, LV_ALIGN_CENTER, 0, 62);
    lv_slider_set_range(sl, 10, 100);
    lv_slider_set_value(sl, ui_power_get_brightness(), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(sl, lv_color_hex(0x22303F), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sl, lv_color_hex(0xF1C40F), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sl, lv_color_white(), LV_PART_KNOB);
    lv_obj_add_event_cb(sl, bright_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *hint = lv_label_create(s_panel);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x4A5560), 0);
    lv_label_set_text(hint, LV_SYMBOL_UP "  para cerrar");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -14);

    refresh();
    ESP_LOGI(TAG, "Panel rapido abierto");
}
