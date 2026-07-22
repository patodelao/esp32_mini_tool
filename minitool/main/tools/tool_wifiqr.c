/*
 * tool_wifiqr.c — Comparte tu red Wi-Fi como código QR.
 *
 * Genera el string estándar de unión Wi-Fi y lo muestra como QR sobre un panel
 * blanco (con margen) para que un teléfono lo escanee y se conecte. Es tu propia
 * red en tu propio dispositivo, mostrada localmente (como el compartir Wi-Fi de
 * Android/iOS).
 */
#include "tool.h"
#include "wifi_manager.h"

#include "lvgl.h"
#include <stdio.h>
#include <string.h>

/* Copia src a dst escapando los caracteres especiales del formato WIFI: */
static void append_escaped(char *dst, size_t dstsz, const char *src)
{
    size_t d = strlen(dst);
    for (size_t i = 0; src[i] && d < dstsz - 2; i++) {
        char c = src[i];
        if (c == '\\' || c == ';' || c == ',' || c == ':' || c == '"') {
            dst[d++] = '\\';
        }
        dst[d++] = c;
    }
    dst[d] = '\0';
}

static void wifiqr_open(lv_obj_t *parent)
{
    char ssid[WIFI_SSID_MAX + 1] = {0};
    char pass[WIFI_PASS_MAX + 1] = {0};
    wifi_manager_get_ssid(ssid, sizeof(ssid));
    wifi_manager_get_pass(pass, sizeof(pass));

    /* Construir el string WIFI:T:WPA;S:<ssid>;P:<pass>;; (escapado) */
    char data[160];
    if (pass[0]) {
        snprintf(data, sizeof(data), "WIFI:T:WPA;S:");
        append_escaped(data, sizeof(data), ssid);
        strlcat(data, ";P:", sizeof(data));
        append_escaped(data, sizeof(data), pass);
        strlcat(data, ";;", sizeof(data));
    } else {
        snprintf(data, sizeof(data), "WIFI:T:nopass;S:");
        append_escaped(data, sizeof(data), ssid);
        strlcat(data, ";;", sizeof(data));
    }

    /* Panel blanco con margen (zona de silencio del QR) */
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 168, 168);
    lv_obj_align(panel, LV_ALIGN_CENTER, 0, -14);
    lv_obj_set_style_bg_color(panel, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(panel, 10, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *qr = lv_qrcode_create(panel, 148, lv_color_black(), lv_color_white());
    lv_obj_center(qr);
    lv_qrcode_update(qr, data, strlen(data));

    /* SSID debajo */
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_label_set_text_fmt(lbl, LV_SYMBOL_WIFI "  %s", ssid);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -28);
}

static void wifiqr_close(void) { }

const tool_t tool_wifiqr = {
    .name = "QR WiFi",
    .icon = LV_SYMBOL_WIFI,
    .accent = 0x9B59B6,
    .open = wifiqr_open,
    .close = wifiqr_close,
};
