/*
 * tool_casa.c — Resumen del hogar de un vistazo.
 *
 * Con 13+ sensores y varias tools, "¿cómo está la casa?" obligaba a saltar
 * entre Dashboard, Sensores, Nodos y Alertas. Esta pantalla junta lo esencial
 * en una: estado del refri, el ambiente (temp/humedad/suelo) con su color de
 * alerta, cuántos nodos están online y la última alerta. Solo lectura.
 *
 * Se reconstruye únicamente cuando algo cambió (firma), así el scroll no se
 * resetea en cada refresco — mismo criterio que la tool Nodos.
 */
#include "tool.h"
#include "ui_theme.h"
#include "sensor_service.h"
#include "sensor_alert.h"
#include "fleet_service.h"
#include "alert_service.h"
#include "ui_notify.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static lv_obj_t   *s_cont = NULL;
static lv_timer_t *s_poll = NULL;
static char        s_sig[512];

/* Magnitudes que cuentan como "ambiente" (lo que uno mira para saber cómo está
 * la casa), separadas de la telemetría de salud (rssi/heap/uptime/...). */
static bool is_env(const char *id)
{
    const char *leaf = sensor_leaf(id);
    return strcmp(leaf, "temp") == 0 || strcmp(leaf, "hum") == 0 || strcmp(leaf, "suelo") == 0;
}

static uint32_t val_color(const char *id, uint32_t age)
{
    if (age > sensor_alert_stale_limit(id)) return UI_DIM;
    switch (sensor_alert_state(id)) {
        case SENSOR_ALERT_LOW:
        case SENSOR_ALERT_HIGH: return UI_ALERT;
        default:                return UI_TEXT;
    }
}

/* Color del punto de estado de un sensor de ambiente (verde ok / rojo alerta /
 * gris viejo), independiente del color del valor (que usa val_color). */
static uint32_t env_dot(const char *id, uint32_t age)
{
    if (age > sensor_alert_stale_limit(id)) return UI_DIM;
    switch (sensor_alert_state(id)) {
        case SENSOR_ALERT_LOW:
        case SENSOR_ALERT_HIGH: return UI_ALERT;
        default:                return UI_OK;
    }
}

/* --- Piezas de UI (tarjetas por sección) ---------------------------------- */

/* Abre una tarjeta de sección con el título en color de acento y la devuelve
 * para colgarle filas. */
static lv_obj_t *card_begin(const char *title, uint32_t accent)
{
    lv_obj_t *card = lv_obj_create(s_cont);
    lv_obj_remove_style_all(card);
    lv_obj_set_width(card, 198);
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_all(card, 9, 0);
    lv_obj_set_style_pad_row(card, 4, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(card);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(accent), 0);
    lv_label_set_text(t, title);
    return card;
}

/* Fila etiqueta/valor dentro de una tarjeta, con punto de estado opcional. */
static void card_row(lv_obj_t *card, const char *left, const char *right,
                     uint32_t vcolor, bool dot, uint32_t dotcolor)
{
    lv_obj_t *c = lv_obj_create(card);
    lv_obj_remove_style_all(c);
    lv_obj_set_width(c, LV_PCT(100));
    lv_obj_set_height(c, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(c, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(c, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(c, 6, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);

    if (dot) {
        lv_obj_t *d = lv_obj_create(c);
        lv_obj_remove_style_all(d);
        lv_obj_set_size(d, 9, 9);
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(dotcolor), 0);
    }

    lv_obj_t *l = lv_label_create(c);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(UI_MUTED), 0);
    lv_label_set_text(l, left);
    lv_obj_set_flex_grow(l, 1);          /* empuja el valor a la derecha */

    lv_obj_t *r = lv_label_create(c);
    lv_obj_set_style_text_font(r, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(r, lv_color_hex(vcolor), 0);
    lv_label_set_text(r, right);
}

/* Texto de ancho completo (envuelto) dentro de una tarjeta (la alerta). */
static void card_line(lv_obj_t *card, const char *text, uint32_t color)
{
    lv_obj_t *l = lv_label_create(card);
    lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(l, LV_PCT(100));
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_label_set_text(l, text);
}

static uint32_t level_color(notify_level_t lv)
{
    switch (lv) {
        case NOTIFY_ALERT:   return UI_ALERT;
        case NOTIFY_WARNING: return UI_WARN;
        case NOTIFY_SUCCESS: return UI_OK;
        default:             return UI_INFO;
    }
}

/* --- Contenido ------------------------------------------------------------ */

static void rebuild(void)
{
    lv_obj_clean(s_cont);

    /* Refri */
    lv_obj_t *c = card_begin("Refri", 0x4AA8FF);
    if (!alert_service_refri_has_data()) {
        card_row(c, "Puerta", "sin datos", UI_DIM, false, 0);
    } else {
        bool open = alert_service_refri_open();
        uint32_t col = open ? UI_ALERT : UI_OK;
        card_row(c, "Puerta", open ? "ABIERTA" : "cerrada", col, true, col);
    }

    /* Ambiente */
    c = card_begin("Ambiente", 0x35D07F);
    int n = sensor_count(), env = 0;
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16], name[40];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;
        if (!is_env(id)) continue;
        sensor_friendly_name(id, name, sizeof(name));
        char rv[24];
        snprintf(rv, sizeof(rv), "%s %s", val, sensor_unit(id));
        card_row(c, name, rv, val_color(id, age), true, env_dot(id, age));
        env++;
    }
    if (env == 0) card_row(c, "", "sin sensores", UI_DIM, false, 0);

    /* Red */
    c = card_begin("Red", 0x1ABC9C);
    int fn = fleet_count(), on = 0;
    for (int i = 0; i < fn; i++) {
        char nid[24];
        bool online = false;
        uint32_t age = 0;
        if (fleet_get(i, nid, sizeof(nid), &online, &age) && online) on++;
    }
    char rb[24];
    snprintf(rb, sizeof(rb), "%d/%d online", on, fn);
    uint32_t netc = (fn > 0 && on == fn) ? UI_OK : (on == 0 ? UI_ALERT : UI_WARN);
    card_row(c, "Nodos", rb, netc, true, netc);

    /* Última alerta */
    c = card_begin("Ultima alerta", 0xE67E22);
    notify_record_t rec;
    if (ui_notify_history_count() > 0 && ui_notify_history_get(0, &rec)) {
        char hora[8] = "--:--";
        if (rec.ts > 0) {
            struct tm tm;
            localtime_r(&rec.ts, &tm);
            snprintf(hora, sizeof(hora), "%02d:%02d", tm.tm_hour, tm.tm_min);
        }
        char txt[120];
        snprintf(txt, sizeof(txt), "%s  %s: %s", hora, rec.source, rec.msg);
        card_line(c, txt, level_color(rec.level));
    } else {
        card_row(c, "", "sin alertas", UI_DIM, false, 0);
    }
}

static void build_sig(char *out, int sz)
{
    int p = 0;
    out[0] = '\0';
    p += snprintf(out + p, sz - p, "r%d%d;",
                  alert_service_refri_open(), alert_service_refri_has_data());
    int n = sensor_count();
    for (int i = 0; i < n && p < sz - 48; i++) {
        char id[SENSOR_ID_MAX], val[16];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;
        if (!is_env(id)) continue;
        int st = (age > sensor_alert_stale_limit(id)) ? 9 : (int)sensor_alert_state(id);
        p += snprintf(out + p, sz - p, "%s=%s:%d;", id, val, st);
    }
    int fn = fleet_count(), on = 0;
    for (int i = 0; i < fn; i++) {
        char nid[24];
        bool online = false;
        uint32_t age = 0;
        if (fleet_get(i, nid, sizeof(nid), &online, &age) && online) on++;
    }
    p += snprintf(out + p, sz - p, "net%d/%d;", on, fn);
    notify_record_t rec;
    if (ui_notify_history_count() > 0 && ui_notify_history_get(0, &rec) && p < sz - 40)
        snprintf(out + p, sz - p, "a%ld", (long)rec.ts);
}

static void poll_cb(lv_timer_t *t)
{
    (void)t;
    char sig[512];
    build_sig(sig, sizeof(sig));
    if (strcmp(sig, s_sig) != 0) {
        strlcpy(s_sig, sig, sizeof(s_sig));
        rebuild();
    }
}

static void casa_open(lv_obj_t *parent)
{
    ui_title(parent, "Casa");

    s_cont = lv_obj_create(parent);
    lv_obj_remove_style_all(s_cont);
    lv_obj_set_size(s_cont, 214, 168);
    lv_obj_align(s_cont, LV_ALIGN_CENTER, 0, 12);
    lv_obj_set_flex_flow(s_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_cont, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_cont, 7, 0);
    lv_obj_set_scroll_dir(s_cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_cont, LV_SCROLLBAR_MODE_OFF);

    s_sig[0] = '\0';
    rebuild();
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void casa_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    s_cont = NULL;
}

const tool_t tool_casa = {
    .name = "Casa",
    .icon = LV_SYMBOL_HOME,
    .accent = 0x35D07F,
    .open = casa_open,
    .close = casa_close,
};
