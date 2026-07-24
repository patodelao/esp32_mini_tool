/*
 * fleet_service.c — Implementación del registro de nodos vía MQTT.
 */
#include "fleet_service.h"
#include "mqtt_hub.h"

#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "fleet";

#define FLEET_FILTER    "labo/nodo/+/status"
#define FLEET_IP_FILTER "labo/nodo/+/ip"
#define MAX_NODES    12
#define ID_MAX       24
#define IP_MAX       16   /* "255.255.255.255" + '\0' */

typedef struct {
    char     id[ID_MAX];
    char     ip[IP_MAX];  /* vacío si el nodo no la publica */
    bool     online;
    uint64_t last_us;   /* último mensaje (esp_timer_get_time) */
    bool     used;
} node_t;

static node_t s_nodes[MAX_NODES];

/* Extrae "<id>" de "labo/nodo/<id>/status" (nivel index 2). */
static bool parse_node_id(const char *topic, int len, char *out, int out_size)
{
    int level = 0, start = 0;
    for (int i = 0; i <= len; i++) {
        if (i == len || topic[i] == '/') {
            if (level == 2) {
                int n = i - start;
                if (n <= 0 || n >= out_size) return false;
                memcpy(out, topic + start, n);
                out[n] = '\0';
                return true;
            }
            level++;
            start = i + 1;
        }
    }
    return false;
}

static node_t *find_or_add(const char *id)
{
    int free_slot = -1;
    for (int i = 0; i < MAX_NODES; i++) {
        if (s_nodes[i].used) {
            if (strcmp(s_nodes[i].id, id) == 0) return &s_nodes[i];
        } else if (free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot < 0) return NULL; /* tabla llena */
    strlcpy(s_nodes[free_slot].id, id, sizeof(s_nodes[free_slot].id));
    s_nodes[free_slot].used = true;
    return &s_nodes[free_slot];
}

static void fleet_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)arg;
    char id[ID_MAX];
    if (!parse_node_id(topic, topic_len, id, sizeof(id))) return;

    node_t *n = find_or_add(id);
    if (!n) return;
    n->online = ((int)strlen("online") == data_len && strncmp(data, "online", data_len) == 0);
    n->last_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Nodo %s -> %s", id, n->online ? "online" : "offline");
}

/* labo/nodo/<id>/ip -> la IP con la que se llega al nodo (p.ej. para OTA). */
static void fleet_ip_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)arg;
    if (data_len == 0) return;   /* borrado de retenido */

    char id[ID_MAX];
    if (!parse_node_id(topic, topic_len, id, sizeof(id))) return;

    node_t *n = find_or_add(id);
    if (!n) return;
    int len = data_len < IP_MAX - 1 ? data_len : IP_MAX - 1;
    memcpy(n->ip, data, len);
    n->ip[len] = '\0';
    ESP_LOGI(TAG, "Nodo %s -> IP %s", id, n->ip);
}

void fleet_service_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;
    mqtt_hub_init();
    mqtt_hub_subscribe(FLEET_FILTER, fleet_cb, NULL);
    mqtt_hub_subscribe(FLEET_IP_FILTER, fleet_ip_cb, NULL);
    ESP_LOGI(TAG, "Fleet suscrito a %s y %s", FLEET_FILTER, FLEET_IP_FILTER);
}

int fleet_count(void)
{
    int c = 0;
    for (int i = 0; i < MAX_NODES; i++) if (s_nodes[i].used) c++;
    return c;
}

bool fleet_get(int idx, char *id, int id_size, bool *online, uint32_t *age_s)
{
    int c = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        if (!s_nodes[i].used) continue;
        if (c == idx) {
            if (id) strlcpy(id, s_nodes[i].id, id_size);
            if (online) *online = s_nodes[i].online;
            if (age_s) *age_s = (uint32_t)((esp_timer_get_time() - s_nodes[i].last_us) / 1000000ULL);
            return true;
        }
        c++;
    }
    return false;
}

void fleet_set_local(const char *id, bool online)
{
    if (!id || !id[0]) return;
    node_t *n = find_or_add(id);
    if (!n) return;
    n->online = online;
    n->last_us = esp_timer_get_time();
}

bool fleet_is_online(const char *id)
{
    for (int i = 0; i < MAX_NODES; i++) {
        if (s_nodes[i].used && strcmp(s_nodes[i].id, id) == 0) return s_nodes[i].online;
    }
    return true;   /* nodo desconocido: no suprimir el aviso */
}

bool fleet_get_ip(int idx, char *ip, int ip_size)
{
    int c = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        if (!s_nodes[i].used) continue;
        if (c == idx) {
            if (!s_nodes[i].ip[0]) return false;
            if (ip) strlcpy(ip, s_nodes[i].ip, ip_size);
            return true;
        }
        c++;
    }
    return false;
}
