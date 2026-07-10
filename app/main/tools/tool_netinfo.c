/*
 * tool_netinfo.c — Monitor de red: SSID, RSSI (con barra), IP, gateway y MAC.
 * Se refresca cada segundo.
 */
#include "tool.h"
#include "wifi_manager.h"

#include <stdio.h>

#include "esp_wifi.h"
#include "esp_netif.h"

static lv_obj_t *s_info_label = NULL;
static lv_obj_t *s_rssi_bar = NULL;
static lv_timer_t *s_timer = NULL;

static void update_cb(lv_timer_t *t)
{
    (void)t;
    char buf[192];

    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    if (wifi_manager_is_connected()) {
        wifi_ap_record_t ap = {0};
        esp_wifi_sta_get_ap_info(&ap);

        esp_netif_ip_info_t ip = {0};
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) esp_netif_get_ip_info(netif, &ip);

        snprintf(buf, sizeof(buf),
                 "%s\n"
                 "RSSI: %d dBm\n"
                 "IP: " IPSTR "\n"
                 "GW: " IPSTR "\n"
                 "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 (const char *)ap.ssid, ap.rssi,
                 IP2STR(&ip.ip), IP2STR(&ip.gw),
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        /* RSSI típico: -30 (excelente) a -90 (inutilizable) → 0..100 */
        int q = (ap.rssi + 90) * 100 / 60;
        if (q < 0) q = 0;
        if (q > 100) q = 100;
        lv_bar_set_value(s_rssi_bar, q, LV_ANIM_ON);
        lv_obj_clear_flag(s_rssi_bar, LV_OBJ_FLAG_HIDDEN);
    } else {
        snprintf(buf, sizeof(buf),
                 "Sin conexi\xC3\xB3n\n\n"
                 "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        lv_obj_add_flag(s_rssi_bar, LV_OBJ_FLAG_HIDDEN);
    }

    lv_label_set_text(s_info_label, buf);
}

static void netinfo_open(lv_obj_t *parent)
{
    lv_obj_t *icon = lv_label_create(parent);
    lv_label_set_text(icon, LV_SYMBOL_LIST);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(icon, lv_color_white(), 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -72);

    s_info_label = lv_label_create(parent);
    lv_obj_set_style_text_color(s_info_label, lv_color_hex(0xBFE0FF), 0);
    lv_obj_set_style_text_align(s_info_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_info_label, "...");
    lv_obj_align(s_info_label, LV_ALIGN_CENTER, 0, -5);

    s_rssi_bar = lv_bar_create(parent);
    lv_obj_set_size(s_rssi_bar, 120, 10);
    lv_obj_align(s_rssi_bar, LV_ALIGN_CENTER, 0, 68);
    lv_bar_set_range(s_rssi_bar, 0, 100);
    lv_obj_set_style_bg_color(s_rssi_bar, lv_color_hex(0x33445A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_rssi_bar, lv_color_hex(0x35D07F), LV_PART_INDICATOR);

    update_cb(NULL);
    s_timer = lv_timer_create(update_cb, 1000, NULL);
}

static void netinfo_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    s_info_label = NULL;
    s_rssi_bar = NULL;
}

const tool_t tool_netinfo = {
    .name = "Red",
    .icon = LV_SYMBOL_LIST,
    .accent = 0x8FA8C8,
    .open = netinfo_open,
    .close = netinfo_close,
};
