/*
 * tool_fleet.c — Panel de nodos (ESP/equipos) del home-lab.
 *
 * Muestra los nodos vistos por fleet_service con su estado. Verde=online,
 * rojo=offline, gris="sin señal" (sin mensajes recientes). Solo lectura.
 */
#include "tool.h"
#include "ui_theme.h"
#include "fleet_service.h"

#include <stdio.h>
#include <string.h>

#define STALE_S 90   /* sin mensajes por más de esto => "sin señal" */

static lv_obj_t *s_cont = NULL;
static lv_obj_t *s_empty = NULL;
static lv_timer_t *s_poll = NULL;
static char s_sig[512];

static void add_row(const char *id, bool online, uint32_t age, const char *ip)
{
    lv_obj_t *lbl = lv_label_create(s_cont);
    lv_label_set_recolor(lbl, true);
    lv_obj_set_width(lbl, LV_PCT(100));
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);

    const char *state; const char *color;
    if (age >= STALE_S)   { state = "sin senal"; color = "7F8C8D"; }
    else if (online)      { state = "online";    color = "35D07F"; }
    else                  { state = "offline";   color = "E74C3C"; }

    /* Segunda línea con la IP (si el nodo la publica): es la que se usa para
     * actualizarlo por OTA cuando <id>.local no resuelve. */
    char buf[128];
    if (ip && ip[0])
        snprintf(buf, sizeof(buf), "#DDE6F0 %s#   #%s %s#\n#5A6B7A %s#", id, color, state, ip);
    else
        snprintf(buf, sizeof(buf), "#DDE6F0 %s#   #%s %s#", id, color, state);
    lv_label_set_text(lbl, buf);
}

static void rebuild(void)
{
    lv_obj_clean(s_cont);
    int n = fleet_count();
    for (int i = 0; i < n; i++) {
        char id[24], ip[16]; bool online; uint32_t age;
        if (!fleet_get(i, id, sizeof(id), &online, &age)) continue;
        if (!fleet_get_ip(i, ip, sizeof(ip))) ip[0] = '\0';
        add_row(id, online, age, ip);
    }
    if (s_empty) {
        if (n == 0) lv_obj_clear_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
        else        lv_obj_add_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
    }
}

static void build_signature(char *out, int size)
{
    int used = 0;
    out[0] = '\0';
    int n = fleet_count();
    for (int i = 0; i < n; i++) {
        char id[24], ip[16]; bool online; uint32_t age;
        if (!fleet_get(i, id, sizeof(id), &online, &age)) continue;
        if (!fleet_get_ip(i, ip, sizeof(ip))) ip[0] = '\0';
        int bucket = (age >= STALE_S) ? 2 : (online ? 1 : 0);
        int w = snprintf(out + used, size - used, "%s:%d:%s;", id, bucket, ip);
        if (w <= 0 || w >= size - used) break;
        used += w;
    }
}

static void poll_cb(lv_timer_t *t)
{
    (void)t;
    char sig[512];
    build_signature(sig, sizeof(sig));
    if (strcmp(sig, s_sig) != 0) {
        strlcpy(s_sig, sig, sizeof(s_sig));
        rebuild();
    }
}

static void fleet_open(lv_obj_t *parent)
{
    ui_title(parent, "Nodos");

    s_cont = lv_obj_create(parent);
    lv_obj_remove_style_all(s_cont);
    lv_obj_set_size(s_cont, 200, 150);
    lv_obj_align(s_cont, LV_ALIGN_CENTER, 0, 12);
    lv_obj_set_flex_flow(s_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_cont, 8, 0);
    lv_obj_set_scroll_dir(s_cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_cont, LV_SCROLLBAR_MODE_AUTO);

    s_empty = lv_label_create(parent);
    lv_obj_set_style_text_font(s_empty, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_empty, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_empty, "Sin nodos\nlabo/nodo/<id>/status");
    lv_obj_set_style_text_align(s_empty, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(s_empty);

    s_sig[0] = '\0';
    rebuild();
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void fleet_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    s_cont = NULL;
    s_empty = NULL;
}

const tool_t tool_fleet = {
    .name = "Nodos",
    .icon = LV_SYMBOL_GPS,
    .accent = 0x1ABC9C,
    .open = fleet_open,
    .close = fleet_close,
};
