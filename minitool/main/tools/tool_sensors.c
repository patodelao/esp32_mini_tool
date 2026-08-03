/*
 * tool_sensors.c — Visor de sensores del home-lab (MQTT).
 *
 * Muestra, para el sensor seleccionado:
 *   - nombre legible ("Suelo Pieza")
 *   - valor actual con unidad inferida del topic (°C, %, dBm, s); se colorea
 *     rojo si está fuera de los umbrales y gris si el sensor dejó de publicar
 *   - récord del día: min / max acumulados (reinicio a medianoche o manual)
 *   - gráfico del histórico corto
 *   - pie: antigüedad de la última lectura, o el motivo de la alerta si la hay
 *
 * Se entra por una LISTA con todos los sensores y su valor; al tocar uno se
 * abre su detalle. Con un solo nodo alcanzaba con ciclar de a uno, pero hoy
 * son 13 sensores entre la pieza, el refri y el propio reloj.
 *
 * Botones inferiores (en el detalle):
 *   - lista: vuelve al listado.
 *   - engranaje: abre el editor de umbrales del sensor (min / max, y para el
 *     suelo también cada cuánto mide el nodo). Se guardan en NVS vía
 *     sensor_alert; para el suelo, además se publican retenidos en
 *     labo/config/<nodo>/suelo/{umbral,histeresis,intervalo} para que el
 *     ESP8266 alerte solo aunque el minitool esté apagado.
 *   - papelera: borra el récord del día del sensor actual. Pide confirmación
 *     (dos toques) para evitar borrados accidentales.
 *   - papelera, pulsación LARGA: olvida el sensor entero (récord, histórico,
 *     umbrales, y su retenido en el broker). Para limpiar el "Crudo" que deja
 *     una calibración, o los fantasmas de un nodo renombrado. También pide
 *     confirmación, con la papelera en rojo y una X.
 */
#include "tool.h"
#include "sensor_service.h"
#include "sensor_alert.h"
#include "ui_theme.h"
#include "mqtt_hub.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* Cuándo se considera "viejo" un sensor sale de sensor_alert_stale_limit():
 * depende del intervalo de muestreo configurado, así un sensor que mide cada
 * 10 min no aparece como caído a los 2 minutos. */

static lv_obj_t *s_id_lbl = NULL;
static lv_obj_t *s_val_lbl = NULL;
static lv_obj_t *s_stats_lbl = NULL;   /* récord del día */
static lv_obj_t *s_chart = NULL;
static lv_chart_series_t *s_ser = NULL;
static lv_obj_t *s_foot_lbl = NULL;    /* índice + antigüedad + ventana */
static lv_obj_t *s_empty = NULL;
static lv_obj_t *s_reset_btn = NULL;
static lv_obj_t *s_reset_lbl = NULL;
static lv_obj_t *s_gear_btn = NULL;    /* abre el editor de umbrales */
static lv_obj_t *s_ovl = NULL;         /* overlay editor de umbrales */
static lv_obj_t *s_ovl_lo = NULL;      /* fila del mínimo */
static lv_obj_t *s_ovl_hi = NULL;      /* fila del máximo */
static lv_obj_t *s_ovl_iv = NULL;      /* fila del intervalo (solo suelo) */
static lv_timer_t *s_poll = NULL;

/* Dos vistas: la LISTA con todos los sensores y el DETALLE de uno.
 *
 * Con un nodo eran 3 sensores y pasarlos con ">" iba bien. Hoy son 13 (aire,
 * suelo y salud de la pieza, el refri y el propio reloj), y encontrar uno a
 * base de toques dejo de tener sentido. */
static lv_obj_t *s_detail_view = NULL;
static lv_obj_t *s_list_view = NULL;
/* Filas de la lista, una por sensor. Los arreglos DEBEN dar para TODOS los
 * sensores posibles (SENSOR_MAX): si fueran más chicos, list_build() caparía
 * s_rows_n por debajo de sensor_count(), la comparación de list_refresh() nunca
 * cuadraría y la lista se reconstruiría en cada refresco (1 Hz), reseteando el
 * scroll — no se podía bajar a ver los de más abajo. */
static lv_obj_t *s_row_val[SENSOR_MAX];    /* label del valor de cada fila     */
static lv_obj_t *s_row_dot[SENSOR_MAX];    /* punto de estado de cada fila     */
static lv_obj_t *s_row_trend[SENSOR_MAX];  /* flecha de tendencia de cada fila */
static int       s_row_sensor[SENSOR_MAX]; /* índice de sensor de cada fila    */
static int       s_rows_n = 0;
static bool      s_detail_mode = false;
static lv_timer_t *s_confirm_tmr = NULL;
static bool s_confirm = false;         /* esperando 2º toque de borrado */
static bool s_confirm_forget = false;  /* el borrado pendiente es "olvidar sensor" */
static int s_sel = 0;

/* Estado del editor de umbrales abierto */
static char          s_ovl_id[SENSOR_ID_MAX];
static sensor_rule_t s_edit;
static float s_step = 1, s_rmin = -100, s_rmax = 100;
static float s_lo_last = 0, s_hi_last = 0;   /* para volver a activar un límite */
/* Modos del gráfico, que se ciclan tocándolo. */
typedef enum { CH_RECENT = 0, CH_HOURS, CH_DAYS, CH_COUNT } chart_mode_t;
static chart_mode_t s_chart_mode = CH_RECENT;
static lv_chart_series_t *s_ser2 = NULL;   /* segunda serie: máximos por día */

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
    s_confirm_forget = false;
    reset_visual_idle();
}

static void confirm_timeout_cb(lv_timer_t *t)
{
    (void)t;
    s_confirm_tmr = NULL;   /* repeat_count 1: LVGL ya lo elimina solo */
    s_confirm = false;
    s_confirm_forget = false;
    reset_visual_idle();
}

static void refresh(void);
static void list_refresh(void);
static void show_detail(bool on);

/* Pone la papelera en modo "confirmá" durante 3 s. 'forget' distingue las dos
 * acciones: borrar el récord (check) u olvidar el sensor (X). */
static void confirm_arm(bool forget)
{
    s_confirm = true;
    s_confirm_forget = forget;
    if (s_reset_btn) lv_obj_set_style_bg_color(s_reset_btn, lv_color_hex(0xB0403A), 0);
    if (s_reset_lbl) lv_label_set_text(s_reset_lbl, forget ? LV_SYMBOL_CLOSE : LV_SYMBOL_OK);
    if (s_confirm_tmr) lv_timer_del(s_confirm_tmr);
    s_confirm_tmr = lv_timer_create(confirm_timeout_cb, 3000, NULL);
    lv_timer_set_repeat_count(s_confirm_tmr, 1);
}

static void reset_cb(lv_event_t *e)
{
    (void)e;
    if (sensor_count() == 0) return;
    if (!s_confirm) {
        confirm_arm(false);            /* 1er toque: pedir confirmación */
    } else {
        /* 2º toque: ejecutar lo que se armó. */
        bool forget = s_confirm_forget;
        confirm_cancel();
        if (forget) {
            sensor_forget(s_sel);
            if (s_sel > 0) s_sel--;    /* el índice se corrió al liberar el slot */
        } else {
            sensor_reset_record(s_sel);
        }
        refresh();
    }
}

/* Pulsación larga: la papelera pasa a ofrecer "olvidar el sensor". */
static void forget_cb(lv_event_t *e)
{
    (void)e;
    if (sensor_count() == 0) return;
    confirm_arm(true);
}

/* -------------------------- Editor de umbrales ---------------------------- */

static bool is_soil(const char *id)
{
    return strcmp(sensor_leaf(id), "suelo") == 0;
}

/* Publica la config del suelo (retenida) para que el nodo alerte por su cuenta
 * aunque el minitool esté apagado:
 * labo/config/<nodo>/suelo/{umbral,histeresis,intervalo}. */
static void publish_soil_cfg(const char *id, const sensor_rule_t *r)
{
    char node[24];
    const char *slash = strrchr(id, '/');
    int nlen = slash ? (int)(slash - id) : 0;
    if (nlen >= (int)sizeof(node)) nlen = (int)sizeof(node) - 1;
    memcpy(node, id, nlen);
    node[nlen] = '\0';

    char topic[72], v[12];

    /* Umbral 0 = el nodo no vigila (el usuario apagó el mínimo). */
    snprintf(topic, sizeof(topic), "labo/config/%s/suelo/umbral", node);
    snprintf(v, sizeof(v), "%.0f", r->lo_on ? r->lo : 0.0f);
    mqtt_hub_publish(topic, v, 1, true);

    snprintf(topic, sizeof(topic), "labo/config/%s/suelo/histeresis", node);
    snprintf(v, sizeof(v), "%.0f", r->hyst);
    mqtt_hub_publish(topic, v, 1, true);

    snprintf(topic, sizeof(topic), "labo/config/%s/suelo/intervalo", node);
    snprintf(v, sizeof(v), "%u", (unsigned)r->sample_s);
    mqtt_hub_publish(topic, v, 1, true);
}

/* Intervalos ofrecidos para medir el suelo. El suelo cambia en horas, así que
 * medir cada pocos minutos alcanza y de paso la sonda se energiza menos (dura
 * más) y hay menos tráfico MQTT. */
static const uint16_t s_intervals[] = { 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600 };
#define INTERVAL_COUNT (sizeof(s_intervals) / sizeof(s_intervals[0]))

/* Índice del intervalo guardado (el más cercano por debajo). */
static int interval_index(uint16_t s)
{
    int best = 1;   /* 10 s, el que trae el nodo de fábrica */
    for (int i = 0; i < (int)INTERVAL_COUNT; i++) if (s_intervals[i] <= s) best = i;
    return best;
}

/* "cada 30 s" / "cada 5 min" / "cada 1 h" */
static void fmt_interval(uint16_t s, char *out, int out_size)
{
    if (s < 60)         snprintf(out, out_size, "cada %u s", (unsigned)s);
    else if (s < 3600)  snprintf(out, out_size, "cada %u min", (unsigned)(s / 60));
    else                snprintf(out, out_size, "cada %u h", (unsigned)(s / 3600));
}

/* Texto de una fila: "Min  25 %" o "Min  --" si ese límite está apagado. */
static void ovl_row_text(lv_obj_t *lbl, const char *tag, bool on, float v)
{
    if (!lbl) return;
    char b[32];
    if (on) snprintf(b, sizeof(b), "%s  %.0f %s", tag, v, sensor_unit(s_ovl_id));
    else    snprintf(b, sizeof(b), "%s  --", tag);
    lv_label_set_text(lbl, b);
    lv_obj_set_style_text_color(lbl, on ? lv_color_white() : lv_color_hex(0x5A6B7A), 0);
}

static void ovl_refresh(void)
{
    ovl_row_text(s_ovl_lo, "Min", s_edit.lo_on, s_edit.lo);
    ovl_row_text(s_ovl_hi, "Max", s_edit.hi_on, s_edit.hi);
    if (s_ovl_iv) {
        char b[24];
        fmt_interval(s_edit.sample_s, b, sizeof(b));
        lv_label_set_text(s_ovl_iv, b);
    }
}

static void ovl_close(void)
{
    if (s_ovl) {
        lv_obj_del(s_ovl);
        s_ovl = NULL;
        s_ovl_lo = s_ovl_hi = s_ovl_iv = NULL;
    }
}

static void ovl_iv_bump(int dir)
{
    int i = interval_index(s_edit.sample_s) + dir;
    if (i < 0) i = 0;
    if (i >= (int)INTERVAL_COUNT) i = INTERVAL_COUNT - 1;
    s_edit.sample_s = s_intervals[i];
    ovl_refresh();
}

static void ovl_iv_minus_cb(lv_event_t *e) { (void)e; ovl_iv_bump(-1); }
static void ovl_iv_plus_cb (lv_event_t *e) { (void)e; ovl_iv_bump(+1); }

/* Ajusta un límite. dir = -1 / +1. Bajar del piso del rango lo apaga; subir
 * desde apagado lo vuelve a encender en su último valor. */
static void ovl_bump(bool is_lo, int dir)
{
    bool  *on   = is_lo ? &s_edit.lo_on : &s_edit.hi_on;
    float *val  = is_lo ? &s_edit.lo    : &s_edit.hi;
    float *last = is_lo ? &s_lo_last    : &s_hi_last;

    if (!*on) {
        if (dir > 0) { *on = true; *val = *last; }   /* reactivar */
        ovl_refresh();
        return;
    }
    float nv = *val + dir * s_step;
    if (nv < s_rmin) {           /* un paso más abajo del piso: apagar */
        *last = *val;
        *on = false;
    } else if (nv > s_rmax) {
        nv = s_rmax;
        *val = nv;
    } else {
        *val = nv;
    }
    /* Mantener min < max si ambos están activos. */
    if (s_edit.lo_on && s_edit.hi_on && s_edit.lo > s_edit.hi - s_step) {
        if (is_lo) s_edit.lo = s_edit.hi - s_step;
        else       s_edit.hi = s_edit.lo + s_step;
    }
    ovl_refresh();
}

static void ovl_lo_minus_cb(lv_event_t *e) { (void)e; ovl_bump(true,  -1); }
static void ovl_lo_plus_cb (lv_event_t *e) { (void)e; ovl_bump(true,  +1); }
static void ovl_hi_minus_cb(lv_event_t *e) { (void)e; ovl_bump(false, -1); }
static void ovl_hi_plus_cb (lv_event_t *e) { (void)e; ovl_bump(false, +1); }

static void refresh(void);

static void ovl_ok_cb(lv_event_t *e)
{
    (void)e;
    sensor_alert_set_rule(s_ovl_id, &s_edit);
    if (is_soil(s_ovl_id)) publish_soil_cfg(s_ovl_id, &s_edit);
    ovl_close();
    refresh();
}

/* Crea una fila "[-]  Tag valor  [+]" centrada en y. Devuelve el label. */
static lv_obj_t *ovl_make_row(lv_obj_t *parent, int y,
                              lv_event_cb_t minus_cb, lv_event_cb_t plus_cb)
{
    lv_obj_t *bm = lv_btn_create(parent);
    lv_obj_set_size(bm, 40, 36);
    lv_obj_align(bm, LV_ALIGN_TOP_MID, -82, y - 18);
    lv_obj_set_style_radius(bm, 18, 0);
    lv_obj_set_style_bg_color(bm, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(bm, minus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bml = lv_label_create(bm);
    lv_label_set_text(bml, LV_SYMBOL_MINUS);
    lv_obj_center(bml);

    lv_obj_t *bp = lv_btn_create(parent);
    lv_obj_set_size(bp, 40, 36);
    lv_obj_align(bp, LV_ALIGN_TOP_MID, 82, y - 18);
    lv_obj_set_style_radius(bp, 18, 0);
    lv_obj_set_style_bg_color(bp, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(bp, plus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bpl = lv_label_create(bp);
    lv_label_set_text(bpl, LV_SYMBOL_PLUS);
    lv_obj_center(bpl);

    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, y - 10);
    return lbl;
}

/* Abre el editor de umbrales del sensor mostrado. */
static void gear_cb(lv_event_t *e)
{
    (void)e;
    if (s_ovl) return;
    if (sensor_count() == 0) return;

    sensor_get(s_sel, s_ovl_id, sizeof(s_ovl_id), NULL, 0, NULL);
    sensor_alert_get_rule(s_ovl_id, &s_edit);
    sensor_alert_edit_hints(s_ovl_id, &s_step, &s_rmin, &s_rmax);

    /* Valor al que volver si el usuario apaga y vuelve a encender un límite
     * (la regla conserva el número aunque el límite esté apagado). */
    s_lo_last = s_edit.lo;
    s_hi_last = s_edit.hi;

    s_ovl = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_ovl);
    lv_obj_set_size(s_ovl, 240, 240);
    lv_obj_center(s_ovl);
    lv_obj_set_style_bg_color(s_ovl, lv_color_hex(0x0A0E12), 0);
    lv_obj_set_style_bg_opa(s_ovl, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_ovl, LV_OBJ_FLAG_SCROLLABLE);

    bool soil = is_soil(s_ovl_id);

    lv_obj_t *t = lv_label_create(s_ovl);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(t, "Umbrales");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, soil ? 18 : 24);

    lv_obj_t *nm = lv_label_create(s_ovl);
    lv_obj_set_style_text_font(nm, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(nm, lv_color_hex(0x8FA8C8), 0);
    char name[40];
    sensor_friendly_name(s_ovl_id, name, sizeof(name));
    lv_label_set_text(nm, name);
    lv_obj_align(nm, LV_ALIGN_TOP_MID, 0, soil ? 36 : 44);

    if (soil) {
        /* Tres filas: min, max y cada cuánto mide el nodo. */
        s_ovl_lo = ovl_make_row(s_ovl, 76,  ovl_lo_minus_cb, ovl_lo_plus_cb);
        s_ovl_hi = ovl_make_row(s_ovl, 116, ovl_hi_minus_cb, ovl_hi_plus_cb);
        s_ovl_iv = ovl_make_row(s_ovl, 156, ovl_iv_minus_cb, ovl_iv_plus_cb);
        lv_obj_set_style_text_color(s_ovl_iv, lv_color_hex(0x8FA8C8), 0);
    } else {
        s_ovl_lo = ovl_make_row(s_ovl, 88,  ovl_lo_minus_cb, ovl_lo_plus_cb);
        s_ovl_hi = ovl_make_row(s_ovl, 136, ovl_hi_minus_cb, ovl_hi_plus_cb);

        lv_obj_t *hint = lv_label_create(s_ovl);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(hint, lv_color_hex(0x5A6B7A), 0);
        lv_label_set_text(hint, "--  =  sin alerta");
        lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 164);
    }
    ovl_refresh();

    lv_obj_t *ok = lv_btn_create(s_ovl);
    lv_obj_set_size(ok, 120, 38);
    lv_obj_align(ok, LV_ALIGN_BOTTOM_MID, 0, -14);
    lv_obj_set_style_radius(ok, 19, 0);
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

    char id[SENSOR_ID_MAX], val[16];
    uint32_t age = 0;
    sensor_get(s_sel, id, sizeof(id), val, sizeof(val), &age);
    bool stale = (age > sensor_alert_stale_limit(id));
    const char *u = sensor_unit(id);
    sensor_alert_state_t st = sensor_alert_state(id);
    bool out_of_range = (st == SENSOR_ALERT_LOW || st == SENSOR_ALERT_HIGH);

    if (s_id_lbl) {
        char name[40];
        sensor_friendly_name(id, name, sizeof(name));
        lv_label_set_text(s_id_lbl, name);
    }

    if (s_gear_btn) lv_obj_clear_flag(s_gear_btn, LV_OBJ_FLAG_HIDDEN);

    /* Valor + unidad: gris si está viejo, rojo si está fuera de umbral. */
    if (s_val_lbl) {
        if (u[0]) lv_label_set_text_fmt(s_val_lbl, "%s %s", val, u);
        else      lv_label_set_text(s_val_lbl, val);
        lv_color_t c = lv_color_white();
        if (stale)             c = lv_color_hex(0x8A949C);
        else if (out_of_range) c = lv_color_hex(0xE74C3C);
        lv_obj_set_style_text_color(s_val_lbl, c, 0);
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

    /* Histórico -> gráfico. Tres modos que se ciclan tocándolo:
     *   reciente : una lectura por punto (últimos 30)
     *   24 h     : un promedio por hora
     *   días     : la banda min-max de cada día cerrado, que es la que dice a
     *              cuánto llegó a bajar el suelo esta semana.
     * Si el modo elegido todavía no tiene datos, se cae al reciente. */
    float h[SENSOR_HIST > SENSOR_HIST_H ? SENSOR_HIST : SENSOR_HIST_H];
    float dmin[SENSOR_DAYS], dmax[SENSOR_DAYS];
    int hn = 0, dn = 0;
    chart_mode_t shown = CH_RECENT;

    if (s_chart_mode == CH_DAYS) {
        dn = sensor_history_days(s_sel, dmin, dmax, SENSOR_DAYS);
        if (dn > 1) shown = CH_DAYS;
    } else if (s_chart_mode == CH_HOURS) {
        hn = sensor_history_hourly(s_sel, h, SENSOR_HIST_H);
        if (hn > 1) shown = CH_HOURS;
    }
    if (shown == CH_RECENT) hn = sensor_history(s_sel, h, SENSOR_HIST);

    char span[16] = "";
    if (s_chart_mode == CH_HOURS) snprintf(span, sizeof(span), shown == CH_HOURS ? "  |  %d h" : "  |  sin horas", hn);
    else if (s_chart_mode == CH_DAYS) snprintf(span, sizeof(span), shown == CH_DAYS ? "  |  %d dias" : "  |  sin dias", dn);

    if (s_chart && s_ser && s_ser2) {
        if (shown == CH_DAYS) {
            /* Dos series: mínimos y máximos de cada día. */
            float mn = dmin[0], mx = dmax[0];
            for (int i = 1; i < dn; i++) {
                if (dmin[i] < mn) mn = dmin[i];
                if (dmax[i] > mx) mx = dmax[i];
            }
            int vmin = (int)floorf(mn * 10.0f), vmax = (int)ceilf(mx * 10.0f);
            if (vmax - vmin < 2) { vmin -= 5; vmax += 5; }
            lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, vmin, vmax);
            lv_chart_set_point_count(s_chart, dn);
            for (int i = 0; i < dn; i++) {
                lv_chart_set_value_by_id(s_chart, s_ser,  i, (int)lroundf(dmin[i] * 10.0f));
                lv_chart_set_value_by_id(s_chart, s_ser2, i, (int)lroundf(dmax[i] * 10.0f));
            }
            lv_chart_set_series_color(s_chart, s_ser,  lv_color_hex(UI_INFO));
            lv_chart_set_series_color(s_chart, s_ser2, lv_color_hex(UI_WARN));
            lv_chart_refresh(s_chart);
        } else if (hn > 0) {
            /* Una sola serie: la segunda se aparta del rango visible. */
            float mn = h[0], mx = h[0];
            for (int i = 1; i < hn; i++) { if (h[i] < mn) mn = h[i]; if (h[i] > mx) mx = h[i]; }
            int vmin = (int)floorf(mn * 10.0f), vmax = (int)ceilf(mx * 10.0f);
            if (vmax - vmin < 2) { vmin -= 5; vmax += 5; }
            lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, vmin, vmax);
            lv_chart_set_point_count(s_chart, hn);
            for (int i = 0; i < hn; i++) {
                lv_chart_set_value_by_id(s_chart, s_ser, i, (int)lroundf(h[i] * 10.0f));
                lv_chart_set_value_by_id(s_chart, s_ser2, i, LV_CHART_POINT_NONE);
            }
            lv_color_t sc = lv_color_hex(UI_OK);
            if (stale)             sc = lv_color_hex(UI_WARN);
            else if (out_of_range) sc = lv_color_hex(UI_ALERT);
            lv_chart_set_series_color(s_chart, s_ser, sc);
            lv_chart_refresh(s_chart);
        }
    }

    /* Pie: índice + antigüedad, o el motivo de la alerta si el valor está
     * fuera de umbral (rojo) / el sensor dejó de publicar (ámbar). */
    if (s_foot_lbl) {
        char foot[64];
        lv_color_t fc = lv_color_hex(0x7F8C8D);

        if (out_of_range) {
            sensor_rule_t r;
            sensor_alert_get_rule(id, &r);
            snprintf(foot, sizeof(foot), "%d/%d  |  %s %.0f %s%s", s_sel + 1, n,
                     st == SENSOR_ALERT_LOW ? "min" : "max",
                     st == SENSOR_ALERT_LOW ? r.lo  : r.hi, u, span);
            fc = lv_color_hex(0xE74C3C);
        } else {
            char age_txt[16];
            fmt_age(age, age_txt, sizeof(age_txt));
            snprintf(foot, sizeof(foot), "%d/%d  |  %s%s%s", s_sel + 1, n,
                     age_txt, stale ? "  (viejo)" : "", span);
            if (stale) fc = lv_color_hex(0xE0A030);
        }
        lv_label_set_text(s_foot_lbl, foot);
        lv_obj_set_style_text_color(s_foot_lbl, fc, 0);
    }
}

static void poll_cb(lv_timer_t *t)
{
    (void)t;
    if (s_detail_mode) refresh();
    else               list_refresh();
}

/* ------------------------------ Vista de lista ---------------------------- */

static void row_click_cb(lv_event_t *e)
{
    s_sel = (int)(intptr_t)lv_event_get_user_data(e);
    confirm_cancel();
    show_detail(true);
}

/* Color del punto de estado: gris viejo / rojo fuera de umbral / verde ok. */
static lv_color_t status_color(const char *id, uint32_t age)
{
    if (age > sensor_alert_stale_limit(id)) return lv_color_hex(0x5A6B7A);
    sensor_alert_state_t st = sensor_alert_state(id);
    if (st == SENSOR_ALERT_LOW || st == SENSOR_ALERT_HIGH) return lv_color_hex(0xE74C3C);
    return lv_color_hex(0x35D07F);
}

/* Tendencia del sensor a partir del histórico reciente: -1 baja / 0 estable /
 * +1 sube. Compara el promedio de las primeras lecturas del histórico con el de
 * las últimas; la zona muerta (2% del rango del sensor) evita que el ruido lo
 * haga temblar. */
static int trend_of(int idx, const char *id)
{
    float h[SENSOR_HIST];
    int hn = sensor_history(idx, h, SENSOR_HIST);
    if (hn < 3) return 0;

    int w = hn / 3;
    if (w < 1) w = 1;
    if (w > 5) w = 5;

    float recent = 0, older = 0;
    for (int i = 0; i < w; i++) { recent += h[hn - 1 - i]; older += h[i]; }
    recent /= w;
    older  /= w;

    float step, rmin, rmax;
    sensor_alert_edit_hints(id, &step, &rmin, &rmax);
    float eps = (rmax - rmin) * 0.02f;
    if (eps < 0.05f) eps = 0.05f;

    float d = recent - older;
    if (d >  eps) return 1;
    if (d < -eps) return -1;
    return 0;
}

/* Refresca una fila en el lugar (valor + color del punto y del valor + flecha
 * de tendencia), sin reconstruir, para no romper el scroll mientras se lee. */
static void row_refresh(int r)
{
    int idx = s_row_sensor[r];
    char id[SENSOR_ID_MAX], val[16];
    uint32_t age = 0;
    if (!sensor_get(idx, id, sizeof(id), val, sizeof(val), &age)) return;

    bool stale = (age > sensor_alert_stale_limit(id));
    sensor_alert_state_t st = sensor_alert_state(id);
    bool oor = (st == SENSOR_ALERT_LOW || st == SENSOR_ALERT_HIGH);

    const char *u = sensor_unit(id);
    if (s_row_val[r]) {
        char buf[24];
        if (u[0]) snprintf(buf, sizeof(buf), "%s %s", val, u);
        else      snprintf(buf, sizeof(buf), "%s", val);
        lv_label_set_text(s_row_val[r], buf);

        lv_color_t vc = lv_color_hex(0xDDE6F0);
        if (stale)    vc = lv_color_hex(0x5A6B7A);
        else if (oor) vc = lv_color_hex(0xE74C3C);
        lv_obj_set_style_text_color(s_row_val[r], vc, 0);
    }
    if (s_row_dot[r]) lv_obj_set_style_bg_color(s_row_dot[r], status_color(id, age), 0);

    /* Flecha: subiendo (cálido) / bajando (frío) / estable (guion gris). Si el
     * sensor está viejo, estable, para no mostrar una tendencia fantasma. */
    if (s_row_trend[r]) {
        int tr = stale ? 0 : trend_of(idx, id);
        const char *sym = (tr > 0) ? LV_SYMBOL_UP : (tr < 0) ? LV_SYMBOL_DOWN : "-";
        lv_color_t tc = (tr > 0) ? lv_color_hex(0xE0894A)
                       : (tr < 0) ? lv_color_hex(0x4AA8FF)
                                  : lv_color_hex(0x4A5866);
        lv_label_set_text(s_row_trend[r], sym);
        lv_obj_set_style_text_color(s_row_trend[r], tc, 0);
    }
}

/* Crea una fila-sensor (píldora con punto de estado + magnitud + valor). */
static void row_create(int sensor_idx, const char *id)
{
    int cap = (int)(sizeof(s_row_val) / sizeof(s_row_val[0]));
    if (s_rows_n >= cap) return;

    lv_obj_t *btn = lv_btn_create(s_list_view);
    lv_obj_set_size(btn, 200, 38);
    lv_obj_set_style_radius(btn, 19, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(UI_CARD_PRESS), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)sensor_idx);

    lv_obj_t *dot = lv_obj_create(btn);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 12, 12);
    lv_obj_align(dot, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(0x35D07F), 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    /* Nombre = magnitud (el 1er token del nombre legible; el nodo ya va en el
     * encabezado, así "Temp Pieza" queda solo "Temp"). */
    char name[40], mag[24];
    sensor_friendly_name(id, name, sizeof(name));
    int j = 0;
    for (; name[j] && name[j] != ' ' && j < (int)sizeof(mag) - 1; j++) mag[j] = name[j];
    mag[j] = '\0';
    lv_obj_t *nm = lv_label_create(btn);
    lv_obj_set_style_text_font(nm, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(nm, lv_color_hex(0xB8C4D0), 0);
    lv_label_set_text(nm, mag);
    lv_obj_align(nm, LV_ALIGN_LEFT_MID, 26, 0);

    lv_obj_t *vl = lv_label_create(btn);
    lv_obj_set_style_text_font(vl, &lv_font_montserrat_16, 0);
    lv_obj_align(vl, LV_ALIGN_RIGHT_MID, -30, 0);   /* deja lugar a la flecha */

    /* Flecha de tendencia, pegada al borde derecho. */
    lv_obj_t *tr = lv_label_create(btn);
    lv_obj_set_style_text_font(tr, &lv_font_montserrat_14, 0);
    lv_label_set_text(tr, "");
    lv_obj_align(tr, LV_ALIGN_RIGHT_MID, -10, 0);

    s_row_val[s_rows_n]    = vl;
    s_row_dot[s_rows_n]    = dot;
    s_row_trend[s_rows_n]  = tr;
    s_row_sensor[s_rows_n] = sensor_idx;
    s_rows_n++;
}

/* Encabezado de un nodo ("Pieza", "Refri", ...). */
static void header_create(const char *node, bool first)
{
    lv_obj_t *hdr = lv_label_create(s_list_view);
    lv_obj_set_width(hdr, 196);
    lv_obj_set_style_text_align(hdr, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hdr, lv_color_hex(0x8FA8C8), 0);
    lv_obj_set_style_pad_left(hdr, 8, 0);
    lv_obj_set_style_pad_top(hdr, first ? 0 : 10, 0);   /* aire entre grupos */
    lv_label_set_text(hdr, sensor_node_label(node));
}

/* Reconstruye la lista AGRUPADA por nodo. Solo cuando cambia la CANTIDAD de
 * sensores; los valores se refrescan en el lugar (row_refresh). */
static void list_build(void)
{
    if (!s_list_view) return;
    lv_obj_clean(s_list_view);
    s_rows_n = 0;

    int n = sensor_count();

    /* Nodos únicos, en orden de aparición. */
    char nodes[SENSOR_MAX][24];
    int nn = 0;
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], node[24];
        if (!sensor_get(i, id, sizeof(id), NULL, 0, NULL)) continue;
        sensor_node_id(id, node, sizeof(node));
        bool seen = false;
        for (int k = 0; k < nn; k++) if (strcmp(nodes[k], node) == 0) { seen = true; break; }
        if (!seen && nn < SENSOR_MAX) { snprintf(nodes[nn], sizeof(nodes[0]), "%s", node); nn++; }
    }

    /* Un encabezado por nodo, seguido de sus sensores. */
    for (int k = 0; k < nn; k++) {
        header_create(nodes[k], k == 0);
        for (int i = 0; i < n; i++) {
            char id[SENSOR_ID_MAX], node[24];
            if (!sensor_get(i, id, sizeof(id), NULL, 0, NULL)) continue;
            sensor_node_id(id, node, sizeof(node));
            if (strcmp(node, nodes[k]) == 0) row_create(i, id);
        }
    }

    for (int r = 0; r < s_rows_n; r++) row_refresh(r);
}

static void list_refresh(void)
{
    if (!s_list_view) return;
    if (s_rows_n != sensor_count()) list_build();
    for (int r = 0; r < s_rows_n; r++) row_refresh(r);

    /* El aviso de "sin sensores" lo apaga tambien la lista (no solo refresh()),
     * si no queda prendido detras de las filas. */
    if (s_empty) {
        if (s_rows_n == 0) lv_obj_clear_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
        else               lv_obj_add_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Alterna entre lista y detalle. */
static void show_detail(bool on)
{
    s_detail_mode = on;
    if (s_detail_view) {
        if (on) lv_obj_clear_flag(s_detail_view, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_add_flag(s_detail_view, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_list_view) {
        if (on) lv_obj_add_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_clear_flag(s_list_view, LV_OBJ_FLAG_HIDDEN);
    }
    if (on) refresh();
    else    list_refresh();
}

static void back_to_list_cb(lv_event_t *e)
{
    (void)e;
    confirm_cancel();
    show_detail(false);
}

static void chart_cb(lv_event_t *e)
{
    (void)e;
    s_chart_mode = (chart_mode_t)((s_chart_mode + 1) % CH_COUNT);
    refresh();
}

static void sensors_open(lv_obj_t *parent)
{
    s_sel = 0;
    s_confirm = false;

    /* La lista: se entra por acá. Agrupada por nodo, con encabezados y filas
     * con punto de estado + magnitud + valor. Scroll vertical normal (el
     * snap-center no pega con los encabezados). */
    s_list_view = lv_obj_create(parent);
    lv_obj_remove_style_all(s_list_view);
    lv_obj_set_size(s_list_view, 240, 240);
    lv_obj_center(s_list_view);
    lv_obj_set_flex_flow(s_list_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_list_view, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_list_view, 6, 0);
    lv_obj_set_style_pad_top(s_list_view, 26, 0);       /* empieza bajo el borde */
    lv_obj_set_style_pad_bottom(s_list_view, 42, 0);    /* y termina sobre el borde */
    lv_obj_set_scroll_dir(s_list_view, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_list_view, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_list_view, LV_OBJ_FLAG_EVENT_BUBBLE);   /* deja salir por gesto */

    /* El detalle vive en su propio contenedor para poder ocultarlo entero. */
    s_detail_view = lv_obj_create(parent);
    lv_obj_remove_style_all(s_detail_view);
    lv_obj_set_size(s_detail_view, 240, 240);
    lv_obj_center(s_detail_view);
    lv_obj_clear_flag(s_detail_view, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_detail_view, LV_OBJ_FLAG_HIDDEN);
    parent = s_detail_view;   /* lo de abajo se construye adentro */

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

    /* Tocar el gráfico alterna entre el histórico reciente y el de 24 h. */
    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, 186, 50);
    lv_obj_align(s_chart, LV_ALIGN_CENTER, 0, -4);
    lv_obj_add_flag(s_chart, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_chart, chart_cb, LV_EVENT_CLICKED, NULL);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(s_chart, 3, 0);
    lv_chart_set_update_mode(s_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_bg_opa(s_chart, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_size(s_chart, 0, LV_PART_INDICATOR); /* sin puntos */
    lv_obj_set_style_line_color(s_chart, lv_color_hex(0x2A3A48), LV_PART_MAIN);
    s_ser  = lv_chart_add_series(s_chart, lv_color_hex(UI_OK), LV_CHART_AXIS_PRIMARY_Y);
    /* Segunda serie: solo se usa en el modo por días (máximos). */
    s_ser2 = lv_chart_add_series(s_chart, lv_color_hex(UI_WARN), LV_CHART_AXIS_PRIMARY_Y);

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
    lv_obj_add_event_cb(btn, back_to_list_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, LV_SYMBOL_LIST);
    lv_obj_center(bl);

    s_gear_btn = lv_btn_create(parent);
    lv_obj_set_size(s_gear_btn, 44, 34);
    lv_obj_align(s_gear_btn, LV_ALIGN_BOTTOM_MID, 0, -32);
    lv_obj_set_style_radius(s_gear_btn, 17, 0);
    lv_obj_set_style_bg_color(s_gear_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_gear_btn, gear_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_gear_btn, LV_OBJ_FLAG_HIDDEN);   /* refresh lo muestra si hay sensores */
    lv_obj_t *gl = lv_label_create(s_gear_btn);
    lv_label_set_text(gl, LV_SYMBOL_SETTINGS);
    lv_obj_center(gl);

    s_reset_btn = lv_btn_create(parent);
    lv_obj_set_size(s_reset_btn, 44, 34);
    lv_obj_align(s_reset_btn, LV_ALIGN_BOTTOM_MID, 52, -32);
    lv_obj_set_style_radius(s_reset_btn, 17, 0);
    lv_obj_set_style_bg_color(s_reset_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_reset_btn, reset_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(s_reset_btn, forget_cb, LV_EVENT_LONG_PRESSED, NULL);
    s_reset_lbl = lv_label_create(s_reset_btn);
    lv_label_set_text(s_reset_lbl, LV_SYMBOL_TRASH);
    lv_obj_center(s_reset_lbl);

    /* De vuelta al contenedor real de la tool: el aviso de "sin sensores" se
     * ve en cualquiera de las dos vistas. */
    parent = lv_obj_get_parent(s_detail_view);

    s_empty = lv_label_create(parent);
    lv_obj_set_style_text_font(s_empty, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_empty, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_empty, "Sin sensores\nlabo/sensor/<id>");
    lv_obj_set_style_text_align(s_empty, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_empty, LV_ALIGN_CENTER, 0, 0);

    list_build();
    show_detail(false);        /* se entra por la lista */
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void sensors_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    if (s_confirm_tmr) { lv_timer_del(s_confirm_tmr); s_confirm_tmr = NULL; }
    ovl_close();
    s_confirm = false;
    s_id_lbl = s_val_lbl = s_stats_lbl = s_chart = s_foot_lbl = s_empty = NULL;
    s_ovl_lo = s_ovl_hi = NULL;
    s_reset_btn = s_reset_lbl = s_gear_btn = NULL;
    s_ser = s_ser2 = NULL;
    s_detail_view = s_list_view = NULL;
    s_rows_n = 0;
    s_detail_mode = false;
}

const tool_t tool_sensors = {
    .name = "Sensores",
    .icon = LV_SYMBOL_EYE_OPEN,
    .accent = 0x35D07F,
    .open = sensors_open,
    .close = sensors_close,
};
