/*
 * tool_wifiscan.c — Escáner de redes Wi-Fi cercanas (SSID + RSSI), en una
 * lista scrolleable. El escaneo es asíncrono para no congelar la UI: el
 * evento SCAN_DONE marca una bandera y un lv_timer (contexto LVGL) recoge
 * los resultados.
 */
#include "tool.h"

#include <stdio.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#define MAX_APS 12

static lv_obj_t *s_list = NULL;
static lv_obj_t *s_status = NULL;
static lv_timer_t *s_timer = NULL;
static volatile bool s_scan_done = false;
static bool s_scanning = false;
static bool s_handler_registered = false;

static void scan_done_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id; (void)data;
    s_scan_done = true;
}

static void start_scan(void)
{
    if (s_scanning) return;
    s_scan_done = false;
    esp_err_t err = esp_wifi_scan_start(NULL, false /* no bloquear */);
    if (err == ESP_OK) {
        s_scanning = true;
        lv_label_set_text(s_status, "Escaneando...");
    } else {
        /* Típico: hay un intento de conexión en curso */
        lv_label_set_text(s_status, "Ocupado, reintenta");
    }
}

static void populate_list(void)
{
    uint16_t n = MAX_APS;
    static wifi_ap_record_t recs[MAX_APS];
    if (esp_wifi_scan_get_ap_records(&n, recs) != ESP_OK) n = 0;

    lv_obj_clean(s_list);
    if (n == 0) {
        lv_label_set_text(s_status, "Sin redes");
        return;
    }

    char buf[48];
    for (int i = 0; i < n; i++) {
        const char *ssid = (const char *)recs[i].ssid;
        snprintf(buf, sizeof(buf), "%s  (%d)",
                 ssid[0] ? ssid : "<oculta>", recs[i].rssi);
        lv_obj_t *btn = lv_list_add_btn(s_list, LV_SYMBOL_WIFI, buf);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_text_color(btn, lv_color_hex(0xBFE0FF), 0);
    }
    lv_label_set_text_fmt(s_status, "%d redes", n);
}

static void tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_scanning && s_scan_done) {
        s_scanning = false;
        populate_list();
    }
}

static void rescan_cb(lv_event_t *e)
{
    (void)e;
    start_scan();
}

static void wifiscan_open(lv_obj_t *parent)
{
    s_status = lv_label_create(parent);
    lv_obj_set_style_text_color(s_status, lv_color_hex(0x8899AA), 0);
    lv_obj_align(s_status, LV_ALIGN_TOP_MID, 0, 46);

    s_list = lv_list_create(parent);
    lv_obj_set_size(s_list, 180, 118);
    lv_obj_align(s_list, LV_ALIGN_CENTER, 0, 8);
    lv_obj_set_style_bg_color(s_list, lv_color_hex(0x0A2436), 0);
    lv_obj_set_style_border_width(s_list, 0, 0);
    lv_obj_set_style_radius(s_list, 12, 0);

    lv_obj_t *rescan = lv_btn_create(parent);
    lv_obj_set_size(rescan, 110, 30);
    lv_obj_align(rescan, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_radius(rescan, 15, 0);
    lv_obj_set_style_bg_color(rescan, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(rescan, rescan_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(rescan);
    lv_label_set_text(lbl, "Re-escanear");
    lv_obj_center(lbl);

    if (!s_handler_registered) {
        esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                   &scan_done_handler, NULL);
        s_handler_registered = true;
    }

    s_timer = lv_timer_create(tick_cb, 300, NULL);
    start_scan();
}

static void wifiscan_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    if (s_handler_registered) {
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                     &scan_done_handler);
        s_handler_registered = false;
    }
    s_scanning = false;
    s_list = NULL;
    s_status = NULL;
}

const tool_t tool_wifiscan = {
    .name = "Scan",
    .icon = LV_SYMBOL_EYE_OPEN,
    .accent = 0x2ED9A3,
    .open = wifiscan_open,
    .close = wifiscan_close,
};
