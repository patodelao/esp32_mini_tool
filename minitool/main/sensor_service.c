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
#include "mqtt_hub.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>

static const char *TAG = "sensor_svc";

#define SENSOR_FILTER "labo/sensor/#"
#define SENSOR_PREFIX "labo/sensor/"
#define MAX_SENSORS   8
#define ID_MAX        32
#define VAL_MAX       16

#define REC_NS   "sensrec"   /* namespace NVS del récord */
#define REC_KEY  "recs"      /* blob con el arreglo de récords */

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

    if (changed) records_save();
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
    record_restore(&s_sensors[slot]);   /* recupera récord de NVS si existe */
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
    ESP_LOGI(TAG, "Sensor %s = %s", id, valbuf);
}

void sensor_service_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;
    records_load();
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

bool sensor_get_record(int idx, float *mn, float *mx, bool *valid)
{
    sensor_t *s = nth(idx);
    if (!s) return false;
    if (mn)    *mn = s->rec_min;
    if (mx)    *mx = s->rec_max;
    if (valid) *valid = s->rec_valid;
    return true;
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
    records_save();
    ESP_LOGI(TAG, "Récord de %s borrado (reinicia en %.1f)", s->id, f);
}
