/*
 * tool_wifi.c — Estado y configuración de Wi-Fi.
 * Conectar/desconectar sin bloquear la UI, y edición de SSID/clave con
 * teclado en pantalla (persisten en NVS vía wifi_manager).
 */
#include "tool.h"
#include "wifi_manager.h"
#include "ui_textinput.h"

static lv_obj_t *s_status_label = NULL;
static lv_obj_t *s_ssid_label = NULL;
static lv_obj_t *s_btn_label = NULL;
static lv_timer_t *s_timer = NULL;

static void refresh(void)
{
    bool connected = wifi_manager_is_connected();
    if (s_status_label) {
        lv_label_set_text(s_status_label, connected ? "Conectado" : "Desconectado");
        lv_obj_set_style_text_color(s_status_label,
                                    connected ? lv_color_hex(0x35D07F) : lv_color_hex(0xFF5A5A), 0);
    }
    if (s_ssid_label) {
        char ssid[WIFI_SSID_MAX + 1];
        wifi_manager_get_ssid(ssid, sizeof(ssid));
        lv_label_set_text_fmt(s_ssid_label, "Red: %s", ssid);
    }
    if (s_btn_label) {
        lv_label_set_text(s_btn_label, wifi_manager_is_connected() ? "Desconectar" : "Conectar");
    }
}

static void refresh_cb(lv_timer_t *t) { (void)t; refresh(); }

static void toggle_cb(lv_event_t *e)
{
    (void)e;
    if (wifi_manager_is_connected()) {
        wifi_manager_disconnect();
    } else {
        wifi_manager_connect();
    }
    refresh();
}

/* ---- edición de credenciales ---- */

static void ssid_saved_cb(const char *text, void *user)
{
    (void)user;
    wifi_manager_set_credentials(text, NULL);
    refresh();
}

static void pass_saved_cb(const char *text, void *user)
{
    (void)user;
    wifi_manager_set_credentials(NULL, text);
    refresh();
}

static void edit_ssid_cb(lv_event_t *e)
{
    (void)e;
    char ssid[WIFI_SSID_MAX + 1];
    wifi_manager_get_ssid(ssid, sizeof(ssid));
    ui_text_prompt("Nombre de red (SSID)", ssid, WIFI_SSID_MAX, false, ssid_saved_cb, NULL);
}

static void edit_pass_cb(lv_event_t *e)
{
    (void)e;
    ui_text_prompt("Clave Wi-Fi", "", WIFI_PASS_MAX, true, pass_saved_cb, NULL);
}

/* Botón pequeño con etiqueta, estilo uniforme. */
static lv_obj_t *small_btn(lv_obj_t *parent, const char *text, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 82, 34);
    lv_obj_set_style_radius(btn, 17, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_center(lbl);
    return btn;
}

static void wifi_open(lv_obj_t *parent)
{
    lv_obj_t *icon = lv_label_create(parent);
    lv_label_set_text(icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(icon, lv_color_white(), 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -62);

    s_status_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, -30);

    s_ssid_label = lv_label_create(parent);
    lv_obj_set_style_text_color(s_ssid_label, lv_color_hex(0xBFE0FF), 0);
    lv_obj_align(s_ssid_label, LV_ALIGN_CENTER, 0, -6);

    lv_obj_t *toggle = lv_btn_create(parent);
    lv_obj_set_size(toggle, 130, 38);
    lv_obj_align(toggle, LV_ALIGN_CENTER, 0, 26);
    lv_obj_set_style_radius(toggle, 19, 0);
    lv_obj_add_event_cb(toggle, toggle_cb, LV_EVENT_CLICKED, NULL);
    s_btn_label = lv_label_create(toggle);
    lv_obj_center(s_btn_label);

    lv_obj_t *b1 = small_btn(parent, "SSID", edit_ssid_cb);
    lv_obj_align(b1, LV_ALIGN_CENTER, -45, 72);
    lv_obj_t *b2 = small_btn(parent, "Clave", edit_pass_cb);
    lv_obj_align(b2, LV_ALIGN_CENTER, 45, 72);

    refresh();
    s_timer = lv_timer_create(refresh_cb, 1000, NULL);
}

static void wifi_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    s_status_label = NULL;
    s_ssid_label = NULL;
    s_btn_label = NULL;
}

const tool_t tool_wifi = {
    .name = "WiFi",
    .icon = LV_SYMBOL_WIFI,
    .open = wifi_open,
    .close = wifi_close,
};
