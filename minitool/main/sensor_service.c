/*
 * sensor_service.c — Implementación del visor de sensores MQTT.
 */
#include "sensor_service.h"
#include "mqtt_hub.h"

#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "sensor_svc";

#define SENSOR_FILTER "labo/sensor/#"
#define SENSOR_PREFIX "labo/sensor/"
#define MAX_SENSORS   8
#define ID_MAX        32
#define VAL_MAX       16

typedef struct {
    char     id[ID_MAX];
    char     val[VAL_MAX];       /* último valor como texto */
    float    hist[SENSOR_HIST];  /* ring buffer */
    int      head;               /* índice del próximo a escribir */
    int      count;              /* nº de puntos válidos */
    uint64_t last_us;
    bool     used;
} sensor_t;

static sensor_t s_sensors[MAX_SENSORS];

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
    return &s_sensors[slot];
}

static void sensor_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)arg;
    int plen = (int)strlen(SENSOR_PREFIX);
    if (topic_len <= plen || strncmp(topic, SENSOR_PREFIX, plen) != 0) return;

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
    ESP_LOGI(TAG, "Sensor %s = %s", id, valbuf);
}

void sensor_service_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;
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
