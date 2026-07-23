/*
 * tool_sensors.c — Visor de sensores del home-lab (MQTT).
 *
 * Muestra, para el sensor seleccionado:
 *   - id del topic (lo que sigue a "labo/sensor/", p.ej. "pieza/temp")
 *   - valor actual con unidad inferida del topic (°C, %, dBm, s)
 *   - récord del día: min / max acumulados (reinicio a medianoche o manual)
 *   - gráfico del histórico corto
 *   - antigüedad de la última lectura ("hace 5 s"); si el sensor dejó de
 *     publicar, el valor se atenúa y el pie se pone en ámbar ("viejo").
 *
 * Botones inferiores:
 *   - ciclar (>): pasa al siguiente sensor.
 *   - engranaje: solo en el sensor de suelo; abre un overlay para fijar el
 *     umbral de riego (%), lo guarda en NVS y lo publica retenido en
 *     labo/config/<nodo>/suelo/umbral para que el ESP8266 alerte solo.
 *   - papelera: borra el récord del día del sensor actual. Pide confirmación
 *     (dos toques) para evitar borrados accidentales.
 */
#include "tool.h"
#include "sensor_service.h"
#include "mqtt_hub.h"

#include "nvs.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* Sobre esta antigüedad (s) sin actualizar, el sensor se considera "viejo".
 * El nodo pieza publica cada 10 s; 60 s deja margen para varios ciclos. */
#define STALE_S 60

static lv_obj_t *s_id_lbl = NULL;
static lv_obj_t *s_val_lbl = NULL;
static lv_obj_t *s_stats_lbl = NULL;   /* récord del día */
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser = NULL;
static lv_obj_t *s_foot_lbl = NULL;    /* índice + antigüedad */
static lv_obj_t *s_empty = NULL;
static lv_obj_t *s_reset_btn = NULL;
static lv_obj_t *s_reset_lbl = NULL;
static lv_obj_t *s_gear_btn = NULL;    /* abre el editor de umbral (solo suelo) */
static lv_obj_t *s_ovl = NULL;         /* overlay editor de umbral */
static lv_obj_t *s_ovl_val = NULL;
static lv_timer_t *s_poll = NULL;
static lv_timer_t *s_confirm_tmr = NULL;
static bool s_confirm = false;         /* esperando 2º toque de borrado */
static int s_sel = 0;
static int s_umbral = 20;              /* % umbral de riego (persistido en NVS) */
static char s_cfg_topic[64];           /* topic de config del sensor mostrado */

/* Unidad inferida de la última componente del id del topic. */
static const char *unit_for(const char *id)
{
    const char *slash = strrchr(id, '/');
    const char *leaf = slash ? slash + 1 : id;
    if (strcmp(leaf, "temp") == 0) return "\xC2\xB0" "C";  /* °C (UTF-8, la fuente incluye 0xB0) */
    if (strcmp(leaf, "hum")  == 0) return "%";
    if (strcmp(leaf, "suelo") == 0) return "%";
    if (strcmp(leaf, "rssi") == 0) return "dBm";
    if (strcmp(leaf, "abierta_seg") == 0) return "s";
    return "";
}

/* Nombre legible del nodo (parte del id antes de '/'). Editá la tabla al
 * agregar nodos; si no está, se muestra el id crudo. */
static const char *node_name(const char *node)
{
    static const struct { const char *id; const char *name; } M[] = {
        { "pieza", "Pieza" },
        { "refri", "Refri" },
    };
    for (unsigned i = 0; i < sizeof(M) / sizeof(M[0]); i++)
        if (strcmp(M[i].id, node) == 0) return M[i].name;
    return node;
}

/* Nombre legible de la magnitud (parte tras '/'). ASCII only: la fuente no
 * tiene acentos ni ñ. */
static const char *mag_name(const char *leaf)
{
    if (strcmp(leaf, "temp")  == 0) return "Temp";
    if (strcmp(leaf, "hum")   == 0) return "Hum";
    if (strcmp(leaf, "suelo") == 0) return "Suelo";
    if (strcmp(leaf, "rssi")  == 0) return "Wifi";
    if (strcmp(leaf, "abierta_seg") == 0) return "Abierta";
    return leaf;
}

/* Compone "<Magnitud> <Nodo>", p.ej. "pieza/temp" -> "Temp Pieza". */
static void friendly_name(const char *id, char *out, int out_size)
{
    const char *slash = strrchr(id, '/');
    if (!slash) { snprintf(out, out_size, "%s", id); return; }

    char node[24];
    int nlen = (int)(slash - id);
    if (nlen >= (int)sizeof(node)) nlen = (int)sizeof(node) - 1;
    memcpy(node, id, nlen);
    node[nlen] = '\0';
    snprintf(out, out_size, "%s %s", mag_name(slash + 1), node_name(node));
}

/* "hace 5 s" / "hace 2 min" / "hace 1 h" en un buffer del llamador. */
static void fmt_age(uint32_t age_s, char *out, int out_size)
{
    if (age_s < 60)        snprintf(out, out_size, "hace %u s", (unsigned)age_s);
    else if (age_s < 3600) snprintf(out, out_size, "hace %u min", (unsigned)(age_s / 60));
    else                   snprintf(out, out_size, "hace %u h", (unsigned)(age_s / 3600));
}

/* --------------------------- Borrado del récord --------------------------- */

static void reset_visual_idle(void)
{
    if (s_reset_btn) lv_obj_set_style_bg_color(s_reset_btn, lv_color_hex(0x33445A), 0);
    if (s_reset_lbl) lv_label_set_text(s_reset_lbl, LV_SYMBOL_TRASH);
}

static void confirm_cancel(void)
{
    if (s_confirm_tmr) { lv_timer_del(s_confirm_tmr); s_confirm_tmr = NULL; }
    s_confirm = false;
    reset_visual_idle();
}

static void confirm_timeout_cb(lv_timer_t *t)
{
    (void)t;
    s_confirm_tmr = NULL;   /* repeat_count 1: LVGL ya lo elimina solo */
    s_confirm = false;
    reset_visual_idle();
}

static void refresh(void);

static void reset_cb(lv_event_t *e)
{
    (void)e;
    if (sensor_count() == 0) return;
    if (!s_confirm) {
        /* 1er toque: pedir confirmación (rojo + check) durante 3 s. */
        s_confirm = true;
        if (s_reset_btn) lv_obj_set_style_bg_color(s_reset_btn, lv_color_hex(0xB0403A), 0);
        if (s_reset_lbl) lv_label_set_text(s_reset_lbl, LV_SYMBOL_OK);
        s_confirm_tmr = lv_timer_create(confirm_timeout_cb, 3000, NULL);
        lv_timer_set_repeat_count(s_confirm_tmr, 1);
    } else {
        /* 2º toque: confirmar borrado. */
        sensor_reset_record(s_sel);
        confirm_cancel();
        refresh();
    }
}

/* ----------------------- Umbral de riego (config) ------------------------- */

static bool is_soil(const char *id)
{
    const char *slash = strrchr(id, '/');
    const char *leaf = slash ? slash + 1 : id;
    return strcmp(leaf, "suelo") == 0;
}

static void umbral_load(void)
{
    nvs_handle_t h;
    if (nvs_open("sensor", NVS_READONLY, &h) == ESP_OK) {
        int32_t v;
        if (nvs_get_i32(h, "umbral_riego", &v) == ESP_OK && v >= 5 && v <= 95)
            s_umbral = (int)v;
        nvs_close(h);
    }
}

static void umbral_save(void)
{
    nvs_handle_t h;
    if (nvs_open("sensor", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "umbral_riego", (int32_t)s_umbral);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void ovl_refresh_val(void)
{
    if (s_ovl_val) lv_label_set_text_fmt(s_ovl_val, "%d %%", s_umbral);
}

static void ovl_close(void)
{
    if (s_ovl) { lv_obj_del(s_ovl); s_ovl = NULL; s_ovl_val = NULL; }
}

static void ovl_minus_cb(lv_event_t *e)
{
    (void)e;
    s_umbral -= 5; if (s_umbral < 5) s_umbral = 5;
    ovl_refresh_val();
}

static void ovl_plus_cb(lv_event_t *e)
{
    (void)e;
    s_umbral += 5; if (s_umbral > 95) s_umbral = 95;
    ovl_refresh_val();
}

static void refresh(void);

static void ovl_ok_cb(lv_event_t *e)
{
    (void)e;
    umbral_save();
    char v[8];
    snprintf(v, sizeof(v), "%d", s_umbral);
    /* Config retenida: el nodo la recibe al conectar (incluso tras reiniciar). */
    mqtt_hub_publish(s_cfg_topic, v, 1, true);
    ovl_close();
    refresh();
}

/* Abre el overlay editor de umbral para el sensor de suelo mostrado. */
static void gear_cb(lv_event_t *e)
{
    (void)e;
    if (s_ovl) return;
    if (sensor_count() == 0) return;

    char id[32];
    sensor_get(s_sel, id, sizeof(id), NULL, 0, NULL);
    if (!is_soil(id)) return;

    /* topic labo/config/<nodo>/suelo/umbral a partir del id "<nodo>/suelo" */
    char node[24];
    const char *slash = strrchr(id, '/');
    int nlen = slash ? (int)(slash - id) : 0;
    if (nlen >= (int)sizeof(node)) nlen = (int)sizeof(node) - 1;
    memcpy(node, id, nlen);
    node[nlen] = '\0';
    snprintf(s_cfg_topic, sizeof(s_cfg_topic), "labo/config/%s/suelo/umbral", node);

    s_ovl = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_ovl);
    lv_obj_set_size(s_ovl, 240, 240);
    lv_obj_center(s_ovl);
    lv_obj_set_style_bg_color(s_ovl, lv_color_hex(0x0A0E12), 0);
    lv_obj_set_style_bg_opa(s_ovl, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_ovl, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(s_ovl);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(t, "Umbral riego");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 40);

    s_ovl_val = lv_label_create(s_ovl);
    lv_obj_set_style_text_font(s_ovl_val, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_ovl_val, lv_color_white(), 0);
    lv_obj_align(s_ovl_val, LV_ALIGN_CENTER, 0, -30);
    ovl_refresh_val();

    lv_obj_t *bm = lv_btn_create(s_ovl);
    lv_obj_set_size(bm, 56, 56);
    lv_obj_align(bm, LV_ALIGN_CENTER, -64, 30);
    lv_obj_set_style_radius(bm, 28, 0);
    lv_obj_set_style_bg_color(bm, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(bm, ovl_minus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bml = lv_label_create(bm);
    lv_label_set_text(bml, LV_SYMBOL_MINUS);
    lv_obj_center(bml);

    lv_obj_t *bp = lv_btn_create(s_ovl);
    lv_obj_set_size(bp, 56, 56);
    lv_obj_align(bp, LV_ALIGN_CENTER, 64, 30);
    lv_obj_set_style_radius(bp, 28, 0);
    lv_obj_set_style_bg_color(bp, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(bp, ovl_plus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bpl = lv_label_create(bp);
    lv_label_set_text(bpl, LV_SYMBOL_PLUS);
    lv_obj_center(bpl);

    lv_obj_t *ok = lv_btn_create(s_ovl);
    lv_obj_set_size(ok, 128, 42);
    lv_obj_align(ok, LV_ALIGN_BOTTOM_MID, 0, -26);
    lv_obj_set_style_radius(ok, 21, 0);
    lv_obj_set_style_bg_color(ok, lv_color_hex(0x35D07F), 0);
    lv_obj_add_event_cb(ok, ovl_ok_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *okl = lv_label_create(ok);
    lv_label_set_text(okl, LV_SYMBOL_OK "  Guardar");
    lv_obj_set_style_text_color(okl, lv_color_hex(0x0A0E12), 0);
    lv_obj_center(okl);
}

/* --------------------------------- Refresh -------------------------------- */

static void refresh(void)
{
    int n = sensor_count();

    if (n == 0) {
        if (s_empty) lv_obj_clear_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
        if (s_chart) lv_obj_add_flag(s_chart, LV_OBJ_FLAG_HIDDEN);
        if (s_val_lbl)   lv_label_set_text(s_val_lbl, "");
        if (s_stats_lbl) lv_label_set_text(s_stats_lbl, "");
        if (s_id_lbl)    lv_label_set_text(s_id_lbl, "Sensores");
        if (s_foot_lbl)  lv_label_set_text(s_foot_lbl, "");
        if (s_gear_btn)  lv_obj_add_flag(s_gear_btn, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (s_empty) lv_obj_add_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
    if (s_chart) lv_obj_clear_flag(s_chart, LV_OBJ_FLAG_HIDDEN);

    if (s_sel >= n) s_sel = 0;

    char id[32], val[16];
    uint32_t age = 0;
    sensor_get(s_sel, id, sizeof(id), val, sizeof(val), &age);
    bool stale = (age > STALE_S);
    const char *u = unit_for(id);

    if (s_id_lbl) {
        char name[40];
        friendly_name(id, name, sizeof(name));
        lv_label_set_text(s_id_lbl, name);
    }

    /* El engranaje (editor de umbral) solo aplica al sensor de suelo. */
    if (s_gear_btn) {
        if (is_soil(id)) lv_obj_clear_flag(s_gear_btn, LV_OBJ_FLAG_HIDDEN);
        else             lv_obj_add_flag(s_gear_btn, LV_OBJ_FLAG_HIDDEN);
    }

    /* Valor + unidad; atenuado si está viejo. */
    if (s_val_lbl) {
        if (u[0]) lv_label_set_text_fmt(s_val_lbl, "%s %s", val, u);
        else      lv_label_set_text(s_val_lbl, val);
        lv_obj_set_style_text_color(s_val_lbl,
            stale ? lv_color_hex(0x8A949C) : lv_color_white(), 0);
    }

    /* Récord del día: min / max acumulados. Formateamos los floats con el
     * snprintf ESTÁNDAR (newlib): el printf de LVGL no soporta %f
     * (LV_SPRINTF_USE_FLOAT=0), y mezclar %f con %s ahí corrompe los varargs
     * y crashea. Se le pasa el string ya armado a lv_label_set_text. */
    if (s_stats_lbl) {
        float rmn = 0, rmx = 0; bool rvalid = false;
        sensor_get_record(s_sel, &rmn, &rmx, &rvalid);
        if (rvalid) {
            char sb[48];
            snprintf(sb, sizeof(sb), "hoy  min %.1f  max %.1f %s", rmn, rmx, u);
            lv_label_set_text(s_stats_lbl, sb);
        } else {
            lv_label_set_text(s_stats_lbl, "");
        }
    }

    /* Histórico -> rango del gráfico. */
    float h[SENSOR_HIST];
    int hn = sensor_history(s_sel, h, SENSOR_HIST);
    if (hn > 0 && s_chart && s_ser) {
        float mn = h[0], mx = h[0];
        for (int i = 1; i < hn; i++) { if (h[i] < mn) mn = h[i]; if (h[i] > mx) mx = h[i]; }
        int vmin = (int)floorf(mn * 10.0f);
        int vmax = (int)ceilf(mx * 10.0f);
        if (vmax - vmin < 2) { vmin -= 5; vmax += 5; } /* evitar rango plano */
        lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, vmin, vmax);
        lv_chart_set_point_count(s_chart, hn);
        for (int i = 0; i < hn; i++) {
            lv_chart_set_value_by_id(s_chart, s_ser, i, (int)lroundf(h[i] * 10.0f));
        }
        lv_chart_set_series_color(s_chart, s_ser,
            stale ? lv_color_hex(0xE0A030) : lv_color_hex(0x35D07F));
        lv_chart_refresh(s_chart);
    }

    /* Pie: índice + antigüedad; ámbar si está viejo. */
    if (s_foot_lbl) {
        char age_txt[16];
        fmt_age(age, age_txt, sizeof(age_txt));
        lv_label_set_text_fmt(s_foot_lbl, "%d/%d  |  %s%s",
            s_sel + 1, n, age_txt, stale ? "  (viejo)" : "");
        lv_obj_set_style_text_color(s_foot_lbl,
            stale ? lv_color_hex(0xE0A030) : lv_color_hex(0x7F8C8D), 0);
    }
}

static void poll_cb(lv_timer_t *t) { (void)t; refresh(); }

static void next_cb(lv_event_t *e)
{
    (void)e;
    confirm_cancel();               /* cancelar borrado pendiente al cambiar */
    int n = sensor_count();
    if (n > 0) s_sel = (s_sel + 1) % n;
    refresh();
}

static void sensors_open(lv_obj_t *parent)
{
    s_sel = 0;
    s_confirm = false;
    umbral_load();

    s_id_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_id_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_id_lbl, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(s_id_lbl, "Sensores");
    lv_obj_align(s_id_lbl, LV_ALIGN_TOP_MID, 0, 14);

    s_val_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_val_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_val_lbl, lv_color_white(), 0);
    lv_label_set_text(s_val_lbl, "");
    lv_obj_align(s_val_lbl, LV_ALIGN_TOP_MID, 0, 34);

    s_stats_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_stats_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_stats_lbl, lv_color_hex(0x9AA5AD), 0);
    lv_label_set_text(s_stats_lbl, "");
    lv_obj_align(s_stats_lbl, LV_ALIGN_TOP_MID, 0, 70);

    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, 186, 50);
    lv_obj_align(s_chart, LV_ALIGN_CENTER, 0, -4);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(s_chart, 3, 0);
    lv_chart_set_update_mode(s_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_bg_opa(s_chart, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_size(s_chart, 0, LV_PART_INDICATOR); /* sin puntos */
    lv_obj_set_style_line_color(s_chart, lv_color_hex(0x2A3A48), LV_PART_MAIN);
    s_ser = lv_chart_add_series(s_chart, lv_color_hex(0x35D07F), LV_CHART_AXIS_PRIMARY_Y);

    s_foot_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_foot_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_foot_lbl, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_foot_lbl, "");
    lv_obj_align(s_foot_lbl, LV_ALIGN_TOP_MID, 0, 150);

    /* Fila inferior: ciclar sensor | engranaje (umbral, solo suelo) | papelera. */
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 44, 34);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, -52, -32);
    lv_obj_set_style_radius(btn, 17, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(btn, next_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, LV_SYMBOL_RIGHT);
    lv_obj_center(bl);

    s_gear_btn = lv_btn_create(parent);
    lv_obj_set_size(s_gear_btn, 44, 34);
    lv_obj_align(s_gear_btn, LV_ALIGN_BOTTOM_MID, 0, -32);
    lv_obj_set_style_radius(s_gear_btn, 17, 0);
    lv_obj_set_style_bg_color(s_gear_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_gear_btn, gear_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_gear_btn, LV_OBJ_FLAG_HIDDEN);   /* refresh lo muestra si es suelo */
    lv_obj_t *gl = lv_label_create(s_gear_btn);
    lv_label_set_text(gl, LV_SYMBOL_SETTINGS);
    lv_obj_center(gl);

    s_reset_btn = lv_btn_create(parent);
    lv_obj_set_size(s_reset_btn, 44, 34);
    lv_obj_align(s_reset_btn, LV_ALIGN_BOTTOM_MID, 52, -32);
    lv_obj_set_style_radius(s_reset_btn, 17, 0);
    lv_obj_set_style_bg_color(s_reset_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_reset_btn, reset_cb, LV_EVENT_CLICKED, NULL);
    s_reset_lbl = lv_label_create(s_reset_btn);
    lv_label_set_text(s_reset_lbl, LV_SYMBOL_TRASH);
    lv_obj_center(s_reset_lbl);

    s_empty = lv_label_create(parent);
    lv_obj_set_style_text_font(s_empty, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_empty, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_empty, "Sin sensores\nlabo/sensor/<id>");
    lv_obj_set_style_text_align(s_empty, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_empty, LV_ALIGN_CENTER, 0, 0);

    refresh();
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void sensors_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    if (s_confirm_tmr) { lv_timer_del(s_confirm_tmr); s_confirm_tmr = NULL; }
    ovl_close();
    s_confirm = false;
    s_id_lbl = s_val_lbl = s_stats_lbl = s_chart = s_foot_lbl = s_empty = NULL;
    s_reset_btn = s_reset_lbl = s_gear_btn = NULL;
    s_ser = NULL;
}

const tool_t tool_sensors = {
    .name = "Sensores",
    .icon = LV_SYMBOL_EYE_OPEN,
    .accent = 0x35D07F,
    .open = sensors_open,
    .close = sensors_close,
};
