/*
 * sensor_alert.c — Implementación del motor de umbrales.
 *
 * Una entrada por sensor (id del topic). El estado se calcula con histéresis:
 *   OK   -> LOW   si v < lo
 *   LOW  -> OK    si v >= lo + hyst
 *   OK   -> HIGH  si v > hi
 *   HIGH -> OK    si v <= hi - hyst
 * y cada transición necesita DEBOUNCE_N lecturas seguidas pidiendo lo mismo,
 * para que un pico suelto del ADC no dispare un aviso.
 *
 * "Sin datos" se vigila aparte, con un timer que mira la antigüedad de la
 * última lectura de cada sensor.
 */
#include "sensor_alert.h"
#include "sensor_service.h"
#include "fleet_service.h"
#include "ui_notify.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "sensor_alert";

#define MAX_RULES    16           /* igual que MAX_SENSORS de sensor_service */
#define DEBOUNCE_N   2            /* lecturas seguidas para confirmar un cambio */
#define REPEAT_US    (30ULL * 60 * 1000000ULL)   /* recordatorio cada 30 min */
#define STALE_MIN_S  90           /* piso del "sin datos" (nodos de 10 s) */
#define TICK_US      (10ULL * 1000000ULL)

#define RULE_NS      "sensrule"
#define RULE_KEY     "rules2"     /* v2: el blob ganó sample_s */

typedef struct {
    char                 id[SENSOR_ID_MAX];
    sensor_rule_t        rule;
    sensor_alert_state_t st;        /* OK / LOW / HIGH (stale va aparte) */
    sensor_alert_state_t pend_st;   /* estado candidato en debounce */
    int                  pend_n;
    uint64_t             last_notify_us;
    float                last_v;
    bool                 has_v;
    bool                 stale;
    bool                 used;
} entry_t;

static entry_t s_e[MAX_RULES];
static esp_timer_handle_t s_tick = NULL;

/* Blob de NVS: solo la regla, el estado se recalcula al arrancar. */
typedef struct {
    char     id[SENSOR_ID_MAX];
    float    lo, hi, hyst;
    uint8_t  flags;      /* bit0 lo_on, bit1 hi_on, bit2 stale_on, bit3 remote_lo */
    uint16_t sample_s;
} rule_blob_t;

#define F_LO     0x01
#define F_HI     0x02
#define F_STALE  0x04
#define F_REMOTE 0x08

/* ------------------------------ Por defecto ------------------------------ */

/* Valores iniciales razonables por magnitud. El usuario los ajusta desde la
 * tool Sensores y quedan guardados. */
static void defaults_for(const char *id, sensor_rule_t *r)
{
    const char *leaf = sensor_leaf(id);
    memset(r, 0, sizeof(*r));
    r->stale_on = true;
    r->hyst     = 1.0f;

    if (strcmp(leaf, "suelo") == 0) {
        /* Riego: bajo 25 % hay que regar. El límite bajo lo vigila el nodo. */
        r->lo_on = true;  r->lo = 25.0f;
        r->hi_on = true;  r->hi = 90.0f;   /* encharcado */
        r->hyst  = 5.0f;
        r->remote_lo = true;
        r->sample_s  = 10;                 /* lo que hace el nodo hoy */
    } else if (strcmp(leaf, "suelo_raw") == 0) {
        /* Crudo del ADC: solo aparece en modo calibración, y deja de publicarse
         * al salir de él. Sin umbrales y sin aviso de "sin datos". */
        r->stale_on = false;
    } else if (strcmp(leaf, "temp") == 0) {
        r->lo_on = true;  r->lo = 5.0f;
        r->hi_on = true;  r->hi = 35.0f;
        r->hyst  = 1.0f;
        r->sample_s = 10;
    } else if (strcmp(leaf, "hum") == 0) {
        r->lo_on = true;  r->lo = 25.0f;
        r->hi_on = true;  r->hi = 80.0f;
        r->hyst  = 3.0f;
        r->sample_s = 10;
    } else if (strcmp(leaf, "rssi") == 0) {
        /* Bajo -85 dBm el enlace se vuelve inestable: sirve para elegir dónde
         * dejar el nodo instalado. */
        r->lo_on = true;  r->lo = -85.0f;
        r->hyst  = 5.0f;
        r->sample_s = 60;              /* telemetría: 1 por minuto */
    } else if (strcmp(leaf, "heap") == 0) {
        /* RAM libre del nodo: si baja sostenidamente hay una fuga. */
        r->lo_on = true;  r->lo = 8.0f;
        r->hyst  = 2.0f;
        r->sample_s = 60;
    } else if (strcmp(leaf, "uptime") == 0) {
        /* Sube siempre y se reinicia al reiniciar el nodo: sin umbrales. */
        r->lo_on = r->hi_on = false;
        r->sample_s = 60;
    } else if (strcmp(leaf, "abierta_seg") == 0) {
        r->hi_on = true;  r->hi = 60.0f;
        r->hyst  = 5.0f;
    }
}

void sensor_alert_edit_hints(const char *id, float *step, float *mn, float *mx)
{
    const char *leaf = sensor_leaf(id);
    float s = 1.0f, a = -100.0f, b = 100.0f;

    if (strcmp(leaf, "suelo") == 0 || strcmp(leaf, "hum") == 0) {
        s = 5.0f;  a = 0.0f;    b = 100.0f;
    } else if (strcmp(leaf, "temp") == 0) {
        s = 1.0f;  a = -20.0f;  b = 60.0f;
    } else if (strcmp(leaf, "rssi") == 0) {
        s = 5.0f;  a = -100.0f; b = -30.0f;
    } else if (strcmp(leaf, "heap") == 0) {
        s = 2.0f;  a = 0.0f;    b = 64.0f;
    } else if (strcmp(leaf, "uptime") == 0) {
        s = 60.0f; a = 0.0f;    b = 100000.0f;
    } else if (strcmp(leaf, "abierta_seg") == 0) {
        s = 10.0f; a = 0.0f;    b = 600.0f;
    }
    if (step) *step = s;
    if (mn)   *mn   = a;
    if (mx)   *mx   = b;
}

/* --------------------------------- NVS ----------------------------------- */

static void rules_load(void)
{
    rule_blob_t buf[MAX_RULES];
    nvs_handle_t h;
    if (nvs_open(RULE_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(buf);
    if (nvs_get_blob(h, RULE_KEY, buf, &sz) == ESP_OK) {
        int n = (int)(sz / sizeof(rule_blob_t));
        if (n > MAX_RULES) n = MAX_RULES;
        for (int i = 0; i < n; i++) {
            if (!buf[i].id[0]) continue;
            entry_t *e = &s_e[i];
            memset(e, 0, sizeof(*e));
            strlcpy(e->id, buf[i].id, sizeof(e->id));
            e->rule.lo        = buf[i].lo;
            e->rule.hi        = buf[i].hi;
            e->rule.hyst      = buf[i].hyst;
            e->rule.lo_on     = (buf[i].flags & F_LO)     != 0;
            e->rule.hi_on     = (buf[i].flags & F_HI)     != 0;
            e->rule.stale_on  = (buf[i].flags & F_STALE)  != 0;
            e->rule.remote_lo = (buf[i].flags & F_REMOTE) != 0;
            e->rule.sample_s  = buf[i].sample_s;
            e->used = true;
        }
    }
    nvs_close(h);
}

static void rules_save(void)
{
    rule_blob_t buf[MAX_RULES];
    int n = 0;
    for (int i = 0; i < MAX_RULES; i++) {
        if (!s_e[i].used) continue;
        memset(&buf[n], 0, sizeof(buf[n]));
        strlcpy(buf[n].id, s_e[i].id, sizeof(buf[n].id));
        buf[n].lo    = s_e[i].rule.lo;
        buf[n].hi    = s_e[i].rule.hi;
        buf[n].hyst  = s_e[i].rule.hyst;
        buf[n].flags = (uint8_t)((s_e[i].rule.lo_on     ? F_LO     : 0) |
                                 (s_e[i].rule.hi_on     ? F_HI     : 0) |
                                 (s_e[i].rule.stale_on  ? F_STALE  : 0) |
                                 (s_e[i].rule.remote_lo ? F_REMOTE : 0));
        buf[n].sample_s = s_e[i].rule.sample_s;
        n++;
    }
    nvs_handle_t h;
    if (nvs_open(RULE_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, RULE_KEY, buf, (size_t)n * sizeof(rule_blob_t));
    nvs_commit(h);
    nvs_close(h);
}

/* ------------------------------- Entradas -------------------------------- */

static entry_t *find_or_add(const char *id)
{
    int slot = -1;
    for (int i = 0; i < MAX_RULES; i++) {
        if (s_e[i].used) {
            if (strcmp(s_e[i].id, id) == 0) return &s_e[i];
        } else if (slot < 0) slot = i;
    }
    if (slot < 0) return NULL;
    entry_t *e = &s_e[slot];
    memset(e, 0, sizeof(*e));
    strlcpy(e->id, id, sizeof(e->id));
    defaults_for(id, &e->rule);
    e->used = true;
    return e;
}

/* --------------------------- Notificaciones ------------------------------ */

/* El nodo del suelo ya avisa por su cuenta del límite bajo (labo/alerta/<id>):
 * silenciamos aquí ese lado para no mostrar dos toasts por lo mismo. */
static bool muted(const entry_t *e, sensor_alert_state_t from, sensor_alert_state_t to)
{
    return e->rule.remote_lo &&
           (to == SENSOR_ALERT_LOW || from == SENSOR_ALERT_LOW);
}

/* Textos con sabor local: para el suelo "seco"/"encharcado" se entiende mejor
 * que "bajo"/"alto". Los floats se arman con el snprintf estándar (newlib): el
 * printf de LVGL no soporta %f. */
static void notify_state(const entry_t *e, sensor_alert_state_t from, float v)
{
    if (muted(e, from, e->st)) return;

    char name[40];
    sensor_friendly_name(e->id, name, sizeof(name));
    const char *u    = sensor_unit(e->id);
    bool        soil = strcmp(sensor_leaf(e->id), "suelo") == 0;

    char msg[80];
    notify_level_t lvl;

    switch (e->st) {
    case SENSOR_ALERT_LOW:
        lvl = NOTIFY_WARNING;
        snprintf(msg, sizeof(msg), "%s: %.1f %s (min %.0f)",
                 soil ? "seco" : "bajo", v, u, e->rule.lo);
        break;
    case SENSOR_ALERT_HIGH:
        lvl = NOTIFY_WARNING;
        snprintf(msg, sizeof(msg), "%s: %.1f %s (max %.0f)",
                 soil ? "encharcado" : "alto", v, u, e->rule.hi);
        break;
    default:
        lvl = NOTIFY_SUCCESS;
        snprintf(msg, sizeof(msg), "normal: %.1f %s", v, u);
        break;
    }
    ui_notify_push(name, lvl, msg);
}

/* --------------------------- Evaluación ---------------------------------- */

/* Estado que corresponde a 'v' partiendo del actual (con histéresis). */
static sensor_alert_state_t evaluate(const entry_t *e, float v)
{
    const sensor_rule_t *r = &e->rule;
    switch (e->st) {
    case SENSOR_ALERT_LOW:
        if (!r->lo_on || v >= r->lo + r->hyst) return SENSOR_ALERT_OK;
        return SENSOR_ALERT_LOW;
    case SENSOR_ALERT_HIGH:
        if (!r->hi_on || v <= r->hi - r->hyst) return SENSOR_ALERT_OK;
        return SENSOR_ALERT_HIGH;
    default:
        if (r->lo_on && v < r->lo) return SENSOR_ALERT_LOW;
        if (r->hi_on && v > r->hi) return SENSOR_ALERT_HIGH;
        return SENSOR_ALERT_OK;
    }
}

void sensor_alert_on_value(const char *id, float v)
{
    entry_t *e = find_or_add(id);
    if (!e) return;

    e->last_v = v;
    e->has_v  = true;
    e->stale  = false;   /* el aviso de "volvió" lo da el tick, por nodo */

    uint64_t now = esp_timer_get_time();
    sensor_alert_state_t want = evaluate(e, v);

    if (want != e->st) {
        /* Confirmar con varias lecturas seguidas antes de avisar. */
        if (want == e->pend_st) e->pend_n++;
        else { e->pend_st = want; e->pend_n = 1; }
        if (e->pend_n < DEBOUNCE_N) return;

        sensor_alert_state_t from = e->st;
        e->st     = want;
        e->pend_n = 0;
        e->last_notify_us = now;
        notify_state(e, from, v);
        ESP_LOGI(TAG, "%s: %d -> %d (%.1f)", e->id, (int)from, (int)want, v);
    } else {
        e->pend_st = e->st;
        e->pend_n  = 0;
        /* Recordatorio mientras siga fuera de rango. */
        if (e->st != SENSOR_ALERT_OK && now - e->last_notify_us >= REPEAT_US) {
            e->last_notify_us = now;
            notify_state(e, e->st, v);
        }
    }
}

/* ------------------------------ Sin datos -------------------------------- */

uint32_t sensor_alert_stale_limit(const char *id)
{
    sensor_rule_t r;
    sensor_alert_get_rule(id, &r);
    /* Tres ciclos perdidos: da margen a un reintento de MQTT sin avisar de más. */
    uint32_t lim = (uint32_t)r.sample_s * 3;
    return lim > STALE_MIN_S ? lim : STALE_MIN_S;
}

/* Un nodo agrupa varios sensores (pieza trae temp, hum, suelo, rssi, uptime y
 * heap). Cuando el nodo se cae o vuelve, TODOS se quedan mudos o vuelven a la
 * vez: avisar por sensor eran 6 toasts por el mismo evento. El estado mudo se
 * sigue por sensor (la tool colorea cada uno), pero la notificación se emite
 * una sola vez por nodo. */
typedef struct {
    char node[24];
    bool used;
    bool mudo;      /* último estado notificado, para detectar el cambio */
    int  vistos;    /* sensores vigilados del nodo, contados en este tick */
    int  mudos;
} node_state_t;

static node_state_t s_nodes[MAX_RULES];

static node_state_t *node_find_or_add(const char *node)
{
    int slot = -1;
    for (int i = 0; i < MAX_RULES; i++) {
        if (s_nodes[i].used) {
            if (strcmp(s_nodes[i].node, node) == 0) return &s_nodes[i];
        } else if (slot < 0) slot = i;
    }
    if (slot < 0) return NULL;
    memset(&s_nodes[slot], 0, sizeof(s_nodes[slot]));
    strlcpy(s_nodes[slot].node, node, sizeof(s_nodes[slot].node));
    s_nodes[slot].used = true;
    return &s_nodes[slot];
}

static void tick_cb(void *arg)
{
    (void)arg;

    /* 1) Refrescar el estado mudo de cada sensor (lo usa la UI). */
    int n = sensor_count();
    for (int i = 0; i < n; i++) {
        char     id[SENSOR_ID_MAX];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), NULL, 0, &age)) continue;

        entry_t *e = find_or_add(id);
        if (!e || !e->has_v || !e->rule.stale_on) continue;
        e->stale = (age > sensor_alert_stale_limit(id));
    }

    /* 2) Contar, por nodo, cuántos de sus sensores vigilados están mudos. */
    for (int k = 0; k < MAX_RULES; k++) s_nodes[k].vistos = s_nodes[k].mudos = 0;

    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], node[24];
        if (!sensor_get(i, id, sizeof(id), NULL, 0, NULL)) continue;

        entry_t *e = find_or_add(id);
        if (!e || !e->has_v || !e->rule.stale_on) continue;

        sensor_node_id(id, node, sizeof(node));
        node_state_t *ns = node_find_or_add(node);
        if (!ns) continue;
        ns->vistos++;
        if (e->stale) ns->mudos++;
    }

    /* 3) El nodo está mudo cuando lo están TODOS sus sensores: si alguno sigue
     * publicando, el nodo está vivo y el silencio es de un sensor puntual (que
     * la tool ya muestra atenuado). Se notifica solo el cambio de estado. */
    for (int k = 0; k < MAX_RULES; k++) {
        if (!s_nodes[k].used || s_nodes[k].vistos == 0) continue;

        bool mudo = (s_nodes[k].mudos == s_nodes[k].vistos);

        /* Un nodo que se declaró offline no está "sin datos": está durmiendo o
         * apagado, y eso ya se ve en la tool Nodos. Avisar de eso sería ruido
         * garantizado con los nodos de bajo consumo, que pasan dormidos casi
         * todo el tiempo. */
        if (mudo && !fleet_is_online(s_nodes[k].node)) {
            s_nodes[k].mudo = false;   /* para avisar si vuelve y se queda mudo */
            continue;
        }

        if (mudo == s_nodes[k].mudo) continue;
        s_nodes[k].mudo = mudo;

        const char *label = sensor_node_label(s_nodes[k].node);
        if (mudo) {
            ui_notify_push(label, NOTIFY_WARNING, "sin datos");
            ESP_LOGW(TAG, "Nodo %s sin datos", s_nodes[k].node);
        } else {
            ui_notify_push(label, NOTIFY_INFO, "volvio a publicar");
            ESP_LOGI(TAG, "Nodo %s volvio", s_nodes[k].node);
        }
    }
}

/* ------------------------------ API pública ------------------------------ */

void sensor_alert_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;

    rules_load();

    const esp_timer_create_args_t args = {
        .callback = tick_cb, .name = "sensor_alert",
    };
    if (esp_timer_create(&args, &s_tick) == ESP_OK) {
        esp_timer_start_periodic(s_tick, TICK_US);
    }
    ESP_LOGI(TAG, "Motor de umbrales listo");
}

void sensor_alert_get_rule(const char *id, sensor_rule_t *out)
{
    if (!out) return;
    for (int i = 0; i < MAX_RULES; i++) {
        if (s_e[i].used && strcmp(s_e[i].id, id) == 0) { *out = s_e[i].rule; return; }
    }
    defaults_for(id, out);
}

void sensor_alert_set_rule(const char *id, const sensor_rule_t *r)
{
    entry_t *e = find_or_add(id);
    if (!e || !r) return;
    e->rule = *r;

    /* Reevaluar de inmediato con el último valor: si el usuario acaba de subir
     * el umbral, el aviso debe salir ya (y sin esperar el debounce). */
    e->st      = SENSOR_ALERT_OK;
    e->pend_st = SENSOR_ALERT_OK;
    e->pend_n  = 0;
    if (e->has_v) {
        sensor_alert_state_t want = evaluate(e, e->last_v);
        if (want != SENSOR_ALERT_OK) {
            e->st = want;
            e->last_notify_us = esp_timer_get_time();
            notify_state(e, SENSOR_ALERT_OK, e->last_v);
        }
    }
    rules_save();
    ESP_LOGI(TAG, "Regla de %s: min %s%.1f  max %s%.1f  hyst %.1f",
             id, r->lo_on ? "" : "(off) ", r->lo,
             r->hi_on ? "" : "(off) ", r->hi, r->hyst);
}

void sensor_alert_forget(const char *id)
{
    for (int i = 0; i < MAX_RULES; i++) {
        if (!s_e[i].used || strcmp(s_e[i].id, id) != 0) continue;
        memset(&s_e[i], 0, sizeof(s_e[i]));
        rules_save();
        ESP_LOGI(TAG, "Regla de %s olvidada", id);
        return;
    }
}

sensor_alert_state_t sensor_alert_state(const char *id)
{
    for (int i = 0; i < MAX_RULES; i++) {
        if (!s_e[i].used || strcmp(s_e[i].id, id) != 0) continue;
        if (s_e[i].stale) return SENSOR_ALERT_STALE;
        return s_e[i].st;
    }
    return SENSOR_ALERT_OK;
}
