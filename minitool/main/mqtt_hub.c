/*
 * mqtt_hub.c — Implementación del cliente MQTT compartido.
 */
#include "mqtt_hub.h"
#include "self_node.h"   /* topic del last-will */

#include "mqtt_client.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "mqtt_hub";

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com"
#define MAX_SUBS        12
#define FILTER_MAX      64

typedef struct {
    char          filter[FILTER_MAX];
    mqtt_hub_cb_t cb;
    void         *arg;
    bool          used;
} sub_t;

static esp_mqtt_client_handle_t s_client = NULL;
static bool  s_connected = false;
static sub_t s_subs[MAX_SUBS];

/* --- Coincidencia de topic con filtro MQTT ('+' un nivel, '#' el resto) --- */
static bool topic_matches(const char *flt, const char *top)
{
    while (*flt && *top) {
        if (*flt == '#') return true;
        if (*flt == '+') {
            flt++;                                   /* consume el '+' */
            while (*top && *top != '/') top++;       /* consume un nivel */
        } else {
            if (*flt != *top) return false;
            flt++;
            top++;
        }
    }
    if (*flt == '#') return true;
    return (*flt == '\0' && *top == '\0');
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)args; (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            s_connected = true;
            ESP_LOGI(TAG, "Conectado; (re)suscribiendo filtros");
            for (int i = 0; i < MAX_SUBS; i++) {
                if (s_subs[i].used) esp_mqtt_client_subscribe(s_client, s_subs[i].filter, 1);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            break;

        case MQTT_EVENT_DATA: {
            /* Copia el topic terminado en '\0' para el matcher */
            char topic[128];
            int n = event->topic_len < (int)sizeof(topic) - 1 ? event->topic_len : (int)sizeof(topic) - 1;
            memcpy(topic, event->topic, n);
            topic[n] = '\0';
            for (int i = 0; i < MAX_SUBS; i++) {
                if (s_subs[i].used && topic_matches(s_subs[i].filter, topic)) {
                    s_subs[i].cb(event->topic, event->topic_len, event->data, event->data_len, s_subs[i].arg);
                }
            }
            break;
        }

        default:
            break;
    }
}

void mqtt_hub_init(void)
{
    if (s_client) return;
    /* Last-Will: si el minitool se cuelga o pierde la red sin despedirse, el
     * broker publica "offline" por él, igual que hacen los nodos. Sin esto, el
     * propio reloj figuraría online para siempre en su tool Nodos. */
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .session.last_will = {
            .topic  = SELF_NODE_STATUS_TOPIC,
            .msg    = "offline",
            .qos    = 1,
            .retain = 1,
        },
    };
    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "No se pudo inicializar el cliente MQTT");
        return;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "Hub MQTT iniciado (broker %s)", MQTT_BROKER_URI);
}

bool mqtt_hub_connected(void) { return s_connected; }

void mqtt_hub_subscribe(const char *filter, mqtt_hub_cb_t cb, void *arg)
{
    if (!filter || !cb) return;
    for (int i = 0; i < MAX_SUBS; i++) {
        if (!s_subs[i].used) {
            strlcpy(s_subs[i].filter, filter, sizeof(s_subs[i].filter));
            s_subs[i].cb = cb;
            s_subs[i].arg = arg;
            s_subs[i].used = true;
            if (s_connected && s_client) esp_mqtt_client_subscribe(s_client, filter, 1);
            return;
        }
    }
    ESP_LOGW(TAG, "Sin espacio para más suscripciones (%s)", filter);
}

int mqtt_hub_publish(const char *topic, const char *payload, int qos, bool retain)
{
    if (!s_client || !topic) return -1;
    return esp_mqtt_client_publish(s_client, topic, payload ? payload : "", 0, qos, retain ? 1 : 0);
}
