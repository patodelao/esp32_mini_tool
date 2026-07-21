/*
 * tool_bt.c — Bluetooth LE: visibilidad on/off y nombre configurable con
 * teclado en pantalla (persiste en NVS vía bt_manager).
 */
#include "tool.h"
#include "bt_manager.h"
#include "ui_textinput.h"

static lv_obj_t *s_status_label = NULL;
static lv_obj_t *s_name_label = NULL;
static lv_obj_t *s_btn_label = NULL;
static lv_timer_t *s_timer = NULL;

static void refresh(void)
{
    bt_state_t st = bt_manager_state();
    const char *txt;
    lv_color_t color;
    switch (st) {
    case BT_STATE_CONNECTED:   txt = "Conectado";    color = lv_color_hex(0x35D07F); break;
    case BT_STATE_ADVERTISING: txt = "Visible";      color = lv_color_hex(0x4AA8FF); break;
    case BT_STATE_UNSUPPORTED: txt = "Sin soporte";  color = lv_color_hex(0xFF5A5A); break;
    default:                   txt = "Apagado";      color = lv_color_hex(0x8899AA); break;
    }
    if (s_status_label) {
        lv_label_set_text(s_status_label, txt);
        lv_obj_set_style_text_color(s_status_label, color, 0);
    }
    if (s_name_label) {
        char name[BT_NAME_MAX + 1];
        bt_manager_get_name(name, sizeof(name));
        lv_label_set_text_fmt(s_name_label, "Nombre: %s", name);
    }
    if (s_btn_label) {
        bool on = (st == BT_STATE_ADVERTISING || st == BT_STATE_CONNECTED);
        lv_label_set_text(s_btn_label, on ? "Apagar" : "Encender");
    }
}

static void refresh_cb(lv_timer_t *t) { (void)t; refresh(); }

static void toggle_cb(lv_event_t *e)
{
    (void)e;
    bt_state_t st = bt_manager_state();
    if (st == BT_STATE_UNSUPPORTED) return;
    if (st == BT_STATE_OFF) {
        bt_manager_start();
    } else {
        bt_manager_stop();
    }
    refresh();
}

static void name_saved_cb(const char *text, void *user)
{
    (void)user;
    bt_manager_set_name(text);
    refresh();
}

static void edit_name_cb(lv_event_t *e)
{
    (void)e;
    char name[BT_NAME_MAX + 1];
    bt_manager_get_name(name, sizeof(name));
    ui_text_prompt("Nombre Bluetooth", name, BT_NAME_MAX, false, name_saved_cb, NULL);
}

static void bt_open(lv_obj_t *parent)
{
    lv_obj_t *icon = lv_label_create(parent);
    lv_label_set_text(icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(icon, lv_color_hex(0x4AA8FF), 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -62);

    s_status_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, -30);

    s_name_label = lv_label_create(parent);
    lv_obj_set_style_text_color(s_name_label, lv_color_hex(0xBFE0FF), 0);
    lv_obj_align(s_name_label, LV_ALIGN_CENTER, 0, -6);

    lv_obj_t *toggle = lv_btn_create(parent);
    lv_obj_set_size(toggle, 130, 38);
    lv_obj_align(toggle, LV_ALIGN_CENTER, 0, 26);
    lv_obj_set_style_radius(toggle, 19, 0);
    lv_obj_add_event_cb(toggle, toggle_cb, LV_EVENT_CLICKED, NULL);
    s_btn_label = lv_label_create(toggle);
    lv_obj_center(s_btn_label);

    lv_obj_t *name_btn = lv_btn_create(parent);
    lv_obj_set_size(name_btn, 110, 34);
    lv_obj_align(name_btn, LV_ALIGN_CENTER, 0, 72);
    lv_obj_set_style_radius(name_btn, 17, 0);
    lv_obj_set_style_bg_color(name_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(name_btn, edit_name_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(name_btn);
    lv_label_set_text(lbl, "Nombre");
    lv_obj_center(lbl);

    refresh();
    s_timer = lv_timer_create(refresh_cb, 1000, NULL);
}

static void bt_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    s_status_label = NULL;
    s_name_label = NULL;
    s_btn_label = NULL;
}

const tool_t tool_bt = {
    .name = "BT",
    .icon = LV_SYMBOL_BLUETOOTH,
    .accent = 0x3E7BFF,
    .open = bt_open,
    .close = bt_close,
};
