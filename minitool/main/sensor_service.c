/*
 * sensor_service.c — Implementación del visor de sensores MQTT.
 *
 * Además del último valor y un histórico corto para graficar, mantiene por
 * sensor un récord de mínimo/máximo "del día":
 *   - se acumula con cada mensaje recibido,
 *   - se reinicia solo al cambiar el día (medianoche local, vía TZ+localtime),
 *   - se puede borrar a mano (sensor_reset_record),
 *   - se persiste en NVS (blob) para sobrevivir reinicios/cortes de luz.
 */
#include "sensor_service.h"
#include "sensor_alert.h"
#include "mqtt_hub.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

static const char *TAG = "sensor_svc";

#define SENSOR_FILTER "labo/sensor/#"
#define SENSOR_PREFIX "labo/sensor/"
/* Un solo nodo puede traer 6 topics (temp, hum, suelo, rssi, uptime, heap),
 * así que 8 quedaba corto en cuanto hay dos nodos. */
#define MAX_SENSORS   16
#define ID_MAX        SENSOR_ID_MAX
#define VAL_MAX       16

#define REC_NS   "sensrec"   /* namespace NVS del récord */
#define REC_KEY  "recs"      /* blob con el arreglo de récords */
#define HOUR_KEY "hours"     /* blob con el histórico horario */
#define DAYS_KEY "days"      /* blob con los récords por día    */

typedef struct {
    char     id[ID_MAX];
    char     val[VAL_MAX];       /* último valor como texto */
    float    hist[SENSOR_HIST];  /* ring buffer */
    int      head;               /* índice del próximo a escribir */
    int      count;              /* nº de puntos válidos */
    uint64_t last_us;
    bool     used;
    /* Récord diario */
    float    rec_min, rec_max;
    int32_t  rec_day;            /* id de día (año*366+yday); -1 = sin fijar */
    bool     rec_valid;          /* hay récord acumulado */
    /* Récords de días cerrados: al cambiar el día, el de ayer se guarda acá */
    float    day_min[SENSOR_DAYS], day_max[SENSOR_DAYS];
    int      day_n;
    /* Histórico largo: un promedio por hora */
    float    h_hist[SENSOR_HIST_H];
    int      h_head, h_count;
    int32_t  h_hour;             /* id de hora en curso; -1 = sin fijar */
    float    h_sum;              /* acumulador de la hora en curso */
    int      h_n;
} sensor_t;

static sensor_t s_sensors[MAX_SENSORS];

/* Copia persistida del récord (se carga una vez y se usa para restaurar cada
 * sensor cuando reaparece por MQTT tras un reinicio). */
typedef struct {
    char    id[ID_MAX];
    float   mn, mx;
    int32_t day;
    uint8_t valid;
} rec_blob_t;

static rec_blob_t s_saved[MAX_SENSORS];
static int        s_saved_n = 0;

/* Copia persistida del histórico horario, con la misma lógica: se carga una
 * vez y cada sensor recupera lo suyo cuando reaparece por MQTT. */
typedef struct {
    char    id[ID_MAX];
    float   h[SENSOR_HIST_H];
    int16_t head, count;
    int32_t hour;
} hour_blob_t;

static hour_blob_t s_hsaved[MAX_SENSORS];
static int         s_hsaved_n = 0;

/* Récords por día, con la misma mecánica: se cargan una vez y cada sensor
 * recupera los suyos cuando reaparece. */
typedef struct {
    char    id[ID_MAX];
    float   mn[SENSOR_DAYS], mx[SENSOR_DAYS];
    int16_t n;
} days_blob_t;

static days_blob_t s_dsaved[MAX_SENSORS];
static int         s_dsaved_n = 0;
static volatile bool s_days_dirty = false;

/* Volcado diferido a NVS.
 *
 * El récord cambia con cada nuevo min/max: con el RSSI oscilando un par de dBm
 * eso serían varias escrituras de flash por minuto. Y el histórico horario se
 * cierra para todos los sensores a la vez, o sea N escrituras del mismo blob
 * seguidas. En vez de escribir en el momento, se marca sucio y un timer vuelca
 * como mucho cada FLUSH_MS. Lo que se arriesga es perder hasta 10 s de récord
 * en un corte de luz, que no vale una flash gastada. */
#define FLUSH_MS 10000
static volatile bool      s_rec_dirty   = false;
static volatile bool      s_hours_dirty = false;
static esp_timer_handle_t s_flush_timer = NULL;

/* --------------------------- Nombres y unidades ------------------------- */

const char *sensor_leaf(const char *id)
{
    const char *slash = strrchr(id, '/');
    return slash ? slash + 1 : id;
}

const char *sensor_unit(const char *id)
{
    const char *leaf = sensor_leaf(id);
    if (strcmp(leaf, "temp")  == 0) return "\xC2\xB0" "C";  /* °C (UTF-8, la fuente incluye 0xB0) */
    if (strcmp(leaf, "hum")   == 0) return "%";
    if (strcmp(leaf, "suelo") == 0) return "%";
    if (strcmp(leaf, "rssi")   == 0) return "dBm";
    if (strcmp(leaf, "uptime") == 0) return "min";
    if (strcmp(leaf, "heap")   == 0) return "kB";
    if (strcmp(leaf, "abierta_seg") == 0) return "s";
    return "";
}

void sensor_node_id(const char *id, char *out, int out_size)
{
    const char *slash = strrchr(id, '/');
    int n = slash ? (int)(slash - id) : (int)strlen(id);
    if (n >= out_size) n = out_size - 1;
    memcpy(out, id, n);
    out[n] = '\0';
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

const char *sensor_node_label(const char *node) { return node_name(node); }

/* Nombre legible de la magnitud (parte tras '/'). */
static const char *mag_name(const char *leaf)
{
    if (strcmp(leaf, "temp")  == 0) return "Temp";
    if (strcmp(leaf, "hum")   == 0) return "Hum";
    if (strcmp(leaf, "suelo") == 0) return "Suelo";
    if (strcmp(leaf, "suelo_raw") == 0) return "Crudo";   /* ADC, en calibracion */
    if (strcmp(leaf, "rssi")   == 0) return "Wifi";
    if (strcmp(leaf, "uptime") == 0) return "Encendido";
    if (strcmp(leaf, "heap")   == 0) return "RAM";
    if (strcmp(leaf, "abierta_seg") == 0) return "Abierta";
    return leaf;
}

void sensor_friendly_name(const char *id, char *out, int out_size)
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

/* ------------------------------- Tiempo -------------------------------- */

/* Id de día local (año*366 + día-del-año). -1 si la hora aún no es válida. */
static int32_t current_day(void)
{
    time_t now = time(NULL);
    if (now < 1600000000) return -1;   /* reloj no sincronizado todavía */
    struct tm tm;
    localtime_r(&now, &tm);
    return (int32_t)(tm.tm_year * 366 + tm.tm_yday);
}

/* Id de hora local, creciente y comparable. -1 si el reloj no está en hora:
 * sin hora válida no tiene sentido acumular por hora. */
static int32_t current_hour(void)
{
    time_t now = time(NULL);
    if (now < 1600000000) return -1;
    struct tm tm;
    localtime_r(&now, &tm);
    return (int32_t)((tm.tm_year * 366 + tm.tm_yday) * 24 + tm.tm_hour);
}

/* ------------------------------- NVS ----------------------------------- */

static void records_load(void)
{
    nvs_handle_t h;
    if (nvs_open(REC_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(s_saved);
    if (nvs_get_blob(h, REC_KEY, s_saved, &sz) == ESP_OK) {
        s_saved_n = (int)(sz / sizeof(rec_blob_t));
    }
    nvs_close(h);
}

/* Vuelca los récords de los sensores presentes a NVS. */
static void records_save(void)
{
    rec_blob_t buf[MAX_SENSORS];
    int n = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (!s_sensors[i].used || !s_sensors[i].rec_valid) continue;
        strlcpy(buf[n].id, s_sensors[i].id, sizeof(buf[n].id));
        buf[n].mn    = s_sensors[i].rec_min;
        buf[n].mx    = s_sensors[i].rec_max;
        buf[n].day   = s_sensors[i].rec_day;
        buf[n].valid = 1;
        n++;
    }
    nvs_handle_t h;
    if (nvs_open(REC_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, REC_KEY, buf, (size_t)n * sizeof(rec_blob_t));
    nvs_commit(h);
    nvs_close(h);
}

/* --------------------------- Histórico horario -------------------------- */

static void hours_load(void)
{
    nvs_handle_t h;
    if (nvs_open(REC_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(s_hsaved);
    if (nvs_get_blob(h, HOUR_KEY, s_hsaved, &sz) == ESP_OK) {
        s_hsaved_n = (int)(sz / sizeof(hour_blob_t));
    }
    nvs_close(h);
}

/* Se llama solo al cerrar una hora (una vez por hora), no en cada lectura. */
static void hours_save(void)
{
    static hour_blob_t buf[MAX_SENSORS];   /* static: son ~2 kB, no van al stack */
    int n = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (!s_sensors[i].used || s_sensors[i].h_count == 0) continue;
        memset(&buf[n], 0, sizeof(buf[n]));
        strlcpy(buf[n].id, s_sensors[i].id, sizeof(buf[n].id));
        memcpy(buf[n].h, s_sensors[i].h_hist, sizeof(buf[n].h));
        buf[n].head  = (int16_t)s_sensors[i].h_head;
        buf[n].count = (int16_t)s_sensors[i].h_count;
        buf[n].hour  = s_sensors[i].h_hour;
        n++;
    }
    nvs_handle_t h;
    if (nvs_open(REC_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, HOUR_KEY, buf, (size_t)n * sizeof(hour_blob_t));
    nvs_commit(h);
    nvs_close(h);
}

static void days_load(void)
{
    nvs_handle_t h;
    if (nvs_open(REC_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(s_dsaved);
    if (nvs_get_blob(h, DAYS_KEY, s_dsaved, &sz) == ESP_OK) {
        s_dsaved_n = (int)(sz / sizeof(days_blob_t));
    }
    nvs_close(h);
}

static void days_save(void)
{
    static days_blob_t buf[MAX_SENSORS];
    int n = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (!s_sensors[i].used || s_sensors[i].day_n == 0) continue;
        memset(&buf[n], 0, sizeof(buf[n]));
        strlcpy(buf[n].id, s_sensors[i].id, sizeof(buf[n].id));
        memcpy(buf[n].mn, s_sensors[i].day_min, sizeof(buf[n].mn));
        memcpy(buf[n].mx, s_sensors[i].day_max, sizeof(buf[n].mx));
        buf[n].n = (int16_t)s_sensors[i].day_n;
        n++;
    }
    nvs_handle_t h;
    if (nvs_open(REC_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, DAYS_KEY, buf, (size_t)n * sizeof(days_blob_t));
    nvs_commit(h);
    nvs_close(h);
}

static void days_restore(sensor_t *s)
{
    for (int i = 0; i < s_dsaved_n; i++) {
        if (strcmp(s_dsaved[i].id, s->id) != 0) continue;
        memcpy(s->day_min, s_dsaved[i].mn, sizeof(s->day_min));
        memcpy(s->day_max, s_dsaved[i].mx, sizeof(s->day_max));
        s->day_n = s_dsaved[i].n;
        if (s->day_n > SENSOR_DAYS) s->day_n = SENSOR_DAYS;
        return;
    }
}

/* Cierra el día: empuja el récord de hoy al historial. */
static void day_close(sensor_t *s)
{
    if (!s->rec_valid) return;
    if (s->day_n < SENSOR_DAYS) {
        s->day_min[s->day_n] = s->rec_min;
        s->day_max[s->day_n] = s->rec_max;
        s->day_n++;
    } else {
        for (int i = 1; i < SENSOR_DAYS; i++) {
            s->day_min[i - 1] = s->day_min[i];
            s->day_max[i - 1] = s->day_max[i];
        }
        s->day_min[SENSOR_DAYS - 1] = s->rec_min;
        s->day_max[SENSOR_DAYS - 1] = s->rec_max;
    }
    s_days_dirty = true;
}

static void hours_restore(sensor_t *s)
{
    for (int i = 0; i < s_hsaved_n; i++) {
        if (strcmp(s_hsaved[i].id, s->id) != 0) continue;
        memcpy(s->h_hist, s_hsaved[i].h, sizeof(s->h_hist));
        s->h_head  = s_hsaved[i].head;
        s->h_count = s_hsaved[i].count;
        s->h_hour  = s_hsaved[i].hour;
        return;
    }
}

/* Acumula la lectura en la hora en curso; al cambiar de hora vuelca el
 * promedio al ring y lo persiste. */
static void hour_update(sensor_t *s, float f)
{
    int32_t hr = current_hour();
    if (hr < 0) return;                 /* sin reloj no hay eje temporal */

    if (s->h_hour < 0) {                /* primera lectura con hora válida */
        s->h_hour = hr;
        s->h_sum  = f;
        s->h_n    = 1;
        return;
    }

    if (hr != s->h_hour) {
        if (s->h_n > 0) {
            s->h_hist[s->h_head] = s->h_sum / (float)s->h_n;
            s->h_head = (s->h_head + 1) % SENSOR_HIST_H;
            if (s->h_count < SENSOR_HIST_H) s->h_count++;
            s_hours_dirty = true;   /* una sola escritura para todos los sensores */
        }
        s->h_hour = hr;
        s->h_sum  = 0;
        s->h_n    = 0;
    }
    s->h_sum += f;
    s->h_n++;
}

/* Restaura el récord guardado que coincida con el id del sensor. */
static void record_restore(sensor_t *s)
{
    for (int i = 0; i < s_saved_n; i++) {
        if (s_saved[i].valid && strcmp(s_saved[i].id, s->id) == 0) {
            s->rec_min   = s_saved[i].mn;
            s->rec_max   = s_saved[i].mx;
            s->rec_day   = s_saved[i].day;
            s->rec_valid = true;
            return;
        }
    }
}

/* Actualiza el récord con un nuevo valor; maneja el reset diario y persiste
 * solo cuando algo cambió (para no desgastar la flash). */
static void record_update(sensor_t *s, float f)
{
    int32_t day = current_day();
    bool changed = false;

    /* Reset diario: si la hora es válida y cambió el día, empezar de nuevo. */
    if (day >= 0 && s->rec_valid && s->rec_day != day) {
        day_close(s);          /* guardar el récord de ayer antes de reiniciar */
        s->rec_valid = false;
    }

    if (!s->rec_valid) {
        s->rec_min = s->rec_max = f;
        s->rec_day = day;              /* puede ser -1 si aún no hay hora */
        s->rec_valid = true;
        changed = true;
    } else {
        if (f < s->rec_min) { s->rec_min = f; changed = true; }
        if (f > s->rec_max) { s->rec_max = f; changed = true; }
        /* Fijar el día en cuanto la hora se vuelve válida. */
        if (day >= 0 && s->rec_day != day) { s->rec_day = day; changed = true; }
    }

    if (changed) s_rec_dirty = true;   /* lo vuelca flush_cb */
}

/* ------------------------------ Sensores ------------------------------- */

static sensor_t *find_or_add(const char *id)
{
    int slot = -1;
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (s_sensors[i].used) {
            if (strcmp(s_sensors[i].id, id) == 0) return &s_sensors[i];
        } else if (slot < 0) slot = i;
    }
    if (slot < 0) return NULL;
    memset(&s_sensors[slot], 0, sizeof(sensor_t));
    strlcpy(s_sensors[slot].id, id, sizeof(s_sensors[slot].id));
    s_sensors[slot].used = true;
    s_sensors[slot].rec_day = -1;
    s_sensors[slot].h_hour  = -1;
    record_restore(&s_sensors[slot]);   /* recupera récord de NVS si existe */
    hours_restore(&s_sensors[slot]);    /* y el histórico horario */
    days_restore(&s_sensors[slot]);     /* y los récords por día */
    return &s_sensors[slot];
}

static void sensor_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)arg;
    int plen = (int)strlen(SENSOR_PREFIX);
    if (topic_len <= plen || strncmp(topic, SENSOR_PREFIX, plen) != 0) return;

    /* Payload vacío = borrado de un mensaje retenido en el broker, no una
     * lectura: ignorarlo para no crear sensores fantasma. */
    if (data_len == 0) return;

    char id[ID_MAX];
    int n = topic_len - plen;
    if (n >= (int)sizeof(id)) n = (int)sizeof(id) - 1;
    memcpy(id, topic + plen, n);
    id[n] = '\0';

    char valbuf[VAL_MAX];
    int vn = data_len < (int)sizeof(valbuf) - 1 ? data_len : (int)sizeof(valbuf) - 1;
    memcpy(valbuf, data, vn);
    valbuf[vn] = '\0';

    sensor_t *s = find_or_add(id);
    if (!s) return;

    strlcpy(s->val, valbuf, sizeof(s->val));
    float f = strtof(valbuf, NULL);
    s->hist[s->head] = f;
    s->head = (s->head + 1) % SENSOR_HIST;
    if (s->count < SENSOR_HIST) s->count++;
    s->last_us = esp_timer_get_time();
    record_update(s, f);
    hour_update(s, f);
    sensor_alert_on_value(id, f);   /* umbrales / histéresis / notificaciones */
    ESP_LOGI(TAG, "Sensor %s = %s", id, valbuf);
}

static void flush_cb(void *arg)
{
    (void)arg;
    if (s_rec_dirty)   { s_rec_dirty   = false; records_save(); }
    if (s_hours_dirty) { s_hours_dirty = false; hours_save();   }
    if (s_days_dirty)  { s_days_dirty  = false; days_save();    }
}

void sensor_service_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;
    records_load();
    hours_load();
    days_load();

    const esp_timer_create_args_t args = { .callback = flush_cb, .name = "sensor_flush" };
    if (esp_timer_create(&args, &s_flush_timer) == ESP_OK) {
        esp_timer_start_periodic(s_flush_timer, (uint64_t)FLUSH_MS * 1000ULL);
    }
    sensor_alert_init();   /* motor de umbrales (se alimenta desde sensor_cb) */
    mqtt_hub_init();
    mqtt_hub_subscribe(SENSOR_FILTER, sensor_cb, NULL);
    ESP_LOGI(TAG, "Sensores suscritos a %s", SENSOR_FILTER);
}

int sensor_count(void)
{
    int c = 0;
    for (int i = 0; i < MAX_SENSORS; i++) if (s_sensors[i].used) c++;
    return c;
}

static sensor_t *nth(int idx)
{
    int c = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (!s_sensors[i].used) continue;
        if (c == idx) return &s_sensors[i];
        c++;
    }
    return NULL;
}

bool sensor_get(int idx, char *id, int id_size, char *val, int val_size, uint32_t *age_s)
{
    sensor_t *s = nth(idx);
    if (!s) return false;
    if (id)  strlcpy(id, s->id, id_size);
    if (val) strlcpy(val, s->val, val_size);
    if (age_s) *age_s = (uint32_t)((esp_timer_get_time() - s->last_us) / 1000000ULL);
    return true;
}

int sensor_history(int idx, float *out, int max)
{
    sensor_t *s = nth(idx);
    if (!s || !out) return 0;
    int n = s->count < max ? s->count : max;
    /* el más viejo está en (head - count); recorrer en orden cronológico */
    int start = (s->head - s->count + SENSOR_HIST) % SENSOR_HIST;
    for (int i = 0; i < n; i++) {
        out[i] = s->hist[(start + i) % SENSOR_HIST];
    }
    return n;
}

int sensor_history_hourly(int idx, float *out, int max)
{
    sensor_t *s = nth(idx);
    if (!s || !out) return 0;
    int n = s->h_count < max ? s->h_count : max;
    int start = (s->h_head - s->h_count + SENSOR_HIST_H) % SENSOR_HIST_H;
    for (int i = 0; i < n; i++) {
        out[i] = s->h_hist[(start + i) % SENSOR_HIST_H];
    }
    return n;
}

int sensor_history_days(int idx, float *mins, float *maxs, int max)
{
    sensor_t *s = nth(idx);
    if (!s || !mins || !maxs) return 0;
    int n = s->day_n < max ? s->day_n : max;
    for (int i = 0; i < n; i++) {
        mins[i] = s->day_min[s->day_n - n + i];
        maxs[i] = s->day_max[s->day_n - n + i];
    }
    return n;
}

bool sensor_get_record(int idx, float *mn, float *mx, bool *valid)
{
    sensor_t *s = nth(idx);
    if (!s) return false;
    if (mn)    *mn = s->rec_min;
    if (mx)    *mx = s->rec_max;
    if (valid) *valid = s->rec_valid;
    return true;
}

void sensor_forget(int idx)
{
    sensor_t *s = nth(idx);
    if (!s) return;

    char id[ID_MAX];
    strlcpy(id, s->id, sizeof(id));

    /* Borrar el retenido del broker: si no, el sensor reaparece en cuanto el
     * minitool se reconecte. Payload vacío + retain = olvidalo. */
    char topic[SENSOR_ID_MAX + 16];
    snprintf(topic, sizeof(topic), SENSOR_PREFIX "%s", id);
    mqtt_hub_publish(topic, "", 1, true);

    sensor_alert_forget(id);            /* su regla y su estado */
    memset(s, 0, sizeof(*s));           /* libera el slot */

    /* Persistir sin él: records_save/hours_save recorren los sensores vivos. */
    records_save();
    hours_save();
    ESP_LOGI(TAG, "Sensor %s olvidado", id);
}

void sensor_reset_record(int idx)
{
    sensor_t *s = nth(idx);
    if (!s) return;
    /* Reinicia el récord desde el último valor conocido. */
    float f = strtof(s->val, NULL);
    s->rec_min = s->rec_max = f;
    s->rec_day = current_day();
    s->rec_valid = true;
    records_save();   /* acción del usuario: se guarda ya, sin diferir */
    ESP_LOGI(TAG, "Récord de %s borrado (reinicia en %.1f)", s->id, f);
}
