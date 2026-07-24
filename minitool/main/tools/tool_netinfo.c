/*
 * tool_netinfo.c — Estado del sistema: red, MQTT, hora y registro.
 *
 * Nació como un monitor de red (SSID, RSSI, IP, MAC) y creció por una razón
 * concreta: el reloj puede verse perfecto —menú andando, pantalla impecable— y
 * estar completamente incomunicado, sin nada que lo explique. Pasó de verdad y
 * costó un rato largo de sniffear MQTT y probar IPs para entender que no estaba
 * ni intentando asociarse.
 *
 * Así que esta pantalla no informa: DIAGNOSTICA. Cada línea responde una
 * pregunta que uno se hace en ese momento:
 *
 *   Wi-Fi   ¿a qué red apunta, y está intentando o está apagado?
 *   MQTT    ¿hay red pero el broker no contesta?
 *   Hora    ¿sincronizó? (sin hora no hay alarmas ni registro)
 *   Datos   ¿el registro histórico está guardando?
 *
 * La distinción entre "buscando" y "apagado" es la que faltaba: "sin conexión"
 * tapaba las dos y son problemas opuestos.
 */
#include "tool.h"
#include "ui_theme.h"
#include "wifi_manager.h"
#include "mqtt_hub.h"
#include "datalog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_wifi.h"
#include "esp_netif.h"

static lv_obj_t   *s_info_label = NULL;
static lv_obj_t   *s_rssi_bar = NULL;
static lv_timer_t *s_timer = NULL;

/* Una fila "etiqueta: valor" con el valor coloreado según esté bien o mal.
 * LVGL permite color por span con "#RRGGBB texto#" si el recolor está activo. */
static int row(char *dst, int left, const char *label, uint32_t color, const char *fmt, ...)
{
    va_list ap;
    char val[64];
    va_start(ap, fmt);
    vsnprintf(val, sizeof(val), fmt, ap);
    va_end(ap);
    return snprintf(dst, left, "%s #%06X %s#\n", label, (unsigned)color, val);
}

static void update_cb(lv_timer_t *t)
{
    (void)t;
    char buf[320];
    int n = 0;

    /* --- Wi-Fi --- */
    char ssid[36] = "";
    wifi_manager_get_ssid(ssid, sizeof(ssid));

    if (wifi_manager_is_connected()) {
        wifi_ap_record_t ap = {0};
        esp_wifi_sta_get_ap_info(&ap);

        esp_netif_ip_info_t ip = {0};
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) esp_netif_get_ip_info(netif, &ip);

        n += row(buf + n, sizeof(buf) - n, "Wi-Fi", UI_OK, "%s", ssid);
        n += snprintf(buf + n, sizeof(buf) - n, IPSTR "  %d dBm\n", IP2STR(&ip.ip), ap.rssi);

        int q = (ap.rssi + 90) * 100 / 60;   /* -90 inutilizable .. -30 excelente */
        if (q < 0) q = 0;
        if (q > 100) q = 100;
        lv_bar_set_value(s_rssi_bar, q, LV_ANIM_ON);
        lv_obj_clear_flag(s_rssi_bar, LV_OBJ_FLAG_HIDDEN);
    } else if (wifi_manager_should_connect()) {
        /* Está intentando: el SSID de acá es la primera pista de por qué falla
         * (clave cambiada, red que no existe, o el relleno de secrets.h). */
        n += row(buf + n, sizeof(buf) - n, "Wi-Fi", UI_WARN, "buscando");
        n += snprintf(buf + n, sizeof(buf) - n, "%s\n", ssid[0] ? ssid : "sin red configurada");
        lv_obj_add_flag(s_rssi_bar, LV_OBJ_FLAG_HIDDEN);
    } else {
        n += row(buf + n, sizeof(buf) - n, "Wi-Fi", UI_ALERT, "apagado");
        n += snprintf(buf + n, sizeof(buf) - n, "conectar desde Red\n");
        lv_obj_add_flag(s_rssi_bar, LV_OBJ_FLAG_HIDDEN);
    }

    /* --- MQTT: con red pero sin broker, todo el home-lab se ve vacío --- */
    bool mq = mqtt_hub_connected();
    n += row(buf + n, sizeof(buf) - n, "MQTT", mq ? UI_OK : UI_ALERT,
             "%s", mq ? "conectado" : "sin broker");

    /* --- Hora: sin ella no hay alarmas, ni récords, ni registro --- */
    time_t now;
    time(&now);
    bool hora_ok = (now > 1600000000);
    if (hora_ok) {
        struct tm tm;
        localtime_r(&now, &tm);
        char h[8];
        snprintf(h, sizeof(h), "%02d:%02d", tm.tm_hour, tm.tm_min);
        n += row(buf + n, sizeof(buf) - n, "Hora", UI_OK, "%s", h);
    } else {
        n += row(buf + n, sizeof(buf) - n, "Hora", UI_WARN, "sin sincronizar");
    }

    /* --- Registro histórico --- */
    if (!datalog_ready()) {
        n += row(buf + n, sizeof(buf) - n, "Datos", UI_ALERT, "sin flash");
    } else {
        size_t kb = datalog_size() / 1024;
        n += row(buf + n, sizeof(buf) - n, "Datos", kb ? UI_OK : UI_MUTED, "%u kB", (unsigned)kb);
    }

    lv_label_set_text(s_info_label, buf);
}

static void netinfo_open(lv_obj_t *parent)
{
    ui_title(parent, "Estado");

    s_info_label = lv_label_create(parent);
    lv_label_set_recolor(s_info_label, true);
    lv_obj_set_style_text_color(s_info_label, lv_color_hex(UI_TEXT), 0);
    lv_obj_set_style_text_align(s_info_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_line_space(s_info_label, 3, 0);
    lv_label_set_text(s_info_label, "...");
    lv_obj_align(s_info_label, LV_ALIGN_CENTER, 0, 2);

    s_rssi_bar = lv_bar_create(parent);
    lv_obj_set_size(s_rssi_bar, 110, 6);
    lv_obj_align(s_rssi_bar, LV_ALIGN_BOTTOM_MID, 0, -34);
    lv_bar_set_range(s_rssi_bar, 0, 100);
    lv_obj_set_style_bg_color(s_rssi_bar, lv_color_hex(UI_CARD), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_rssi_bar, lv_color_hex(UI_OK), LV_PART_INDICATOR);

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
    .name = "Estado",
    .icon = LV_SYMBOL_LIST,
    .accent = 0x8FA8C8,
    .open = netinfo_open,
    .close = netinfo_close,
    .hidden = true,   /* vive dentro de la tool Config */
};
