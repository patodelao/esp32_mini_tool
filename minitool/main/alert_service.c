/*
 * alert_service.c — Servicio de alertas MQTT en segundo plano.
 *
 * Mantiene un cliente MQTT persistente y una tabla modular de fuentes. Cada
 * fuente representa un equipo (topic + textos). Al detectar una transición de
 * estado, emite una notificación flotante vía ui_notify.
 *
 * Watchdog por fuente: si el equipo publica su estado "activo" como heartbeat
 * (p.ej. el refri manda "ABIERTO" cada pocos segundos), la ausencia de mensajes
 * durante 'watchdog_ms' se interpreta como vuelta a normal (silenciosa).
 */
#include "alert_service.h"
#include "ui_notify.h"

#include "mqtt_client.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "alert_svc";

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com"

/* --- Definición de una fuente (equipo) ------------------------------------ */

typedef struct {
    /* Configuración (constante) */
    const char *topic;        /* topic MQTT a suscribir                     */
    const char *name;         /* nombre mostrado en la notificación         */
    const char *active_word;  /* payload que significa "en alerta"          */
    const char *msg_active;   /* texto de la notificación al activarse       */
    const char *msg_normal;   /* texto al volver a normal (NULL = no avisar) */
    uint32_t    watchdog_ms;  /* 0 = sin watchdog                            */

    /* Estado en runtime */
    bool               active;
    bool               has_data;
    esp_timer_handle_t wd_timer;
} alert_source_t;

/*
 * TABLA DE FUENTES — para añadir un equipo nuevo, agrega una entrada aquí.
 * Índice 0 reservado al refri (getters de conveniencia).
 */
static alert_source_t s_sources[] = {
    {
        .topic       = "proyectos/casa/refri/puerta",
        .name        = "Refri",
        .active_word = "ABIERTO",
        .msg_active  = "Puerta abierta",
        .msg_normal  = "Puerta cerrada",
        .watchdog_ms = 12000,
    },
};
#define SOURCE_COUNT (sizeof(s_sources) / sizeof(s_sources[0]))
#define SRC_REFRI 0

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_mqtt_connected = false;

/* --- Watchdog -------------------------------------------------------------- */

static void watchdog_cb(void *arg)
{
    alert_source_t *s = (alert_source_t *)arg;
    if (s->active) {
        ESP_LOGW(TAG, "[%s] watchdog: sin heartbeat, se asume normal", s->name);
        s->active = false; /* vuelta a normal silenciosa (pérdida de señal) */
    }
}

static void watchdog_arm(alert_source_t *s)
{
    if (!s->watchdog_ms) return;
    if (!s->wd_timer) {
        const esp_timer_create_args_t args = { .callback = watchdog_cb, .arg = s, .name = "alert_wd" };
        if (esp_timer_create(&args, &s->wd_timer) != ESP_OK) return;
    }
    esp_timer_stop(s->wd_timer); /* re-armar limpio */
    esp_timer_start_once(s->wd_timer, (uint64_t)s->watchdog_ms * 1000ULL);
}

static void watchdog_disarm(alert_source_t *s)
{
    if (s->wd_timer) esp_timer_stop(s->wd_timer);
}

/* --- Procesamiento de mensajes -------------------------------------------- */

static void handle_message(alert_source_t *s, const char *data, int len)
{
    s->has_data = true;
    bool now_active = ((int)strlen(s->active_word) == len &&
                       strncmp(data, s->active_word, len) == 0);

    if (now_active) {
        if (!s->active) {
            s->active = true;
            ui_notify_push(s->name, NOTIFY_ALERT, s->msg_active);
        }
        watchdog_arm(s); /* cada heartbeat re-arma el watchdog */
    } else {
        if (s->active) {
            s->active = false;
            if (s->msg_normal) ui_notify_push(s->name, NOTIFY_SUCCESS, s->msg_normal);
        }
        watchdog_disarm(s);
    }
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)args; (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            s_mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT conectado; suscribiendo %d fuente(s)", (int)SOURCE_COUNT);
            for (size_t i = 0; i < SOURCE_COUNT; i++) {
                esp_mqtt_client_subscribe(s_client, s_sources[i].topic, 1);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_connected = false;
            break;

        case MQTT_EVENT_DATA:
            for (size_t i = 0; i < SOURCE_COUNT; i++) {
                alert_source_t *s = &s_sources[i];
                if ((int)strlen(s->topic) == event->topic_len &&
                    strncmp(event->topic, s->topic, event->topic_len) == 0) {
                    handle_message(s, event->data, event->data_len);
                    break;
                }
            }
            break;

        default:
            break;
    }
}

/* --- API pública ----------------------------------------------------------- */

void alert_service_init(void)
{
    if (s_client) return; /* idempotente */

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "No se pudo inicializar el cliente MQTT");
        return;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client); /* reintenta solo hasta que haya Wi-Fi */
    ESP_LOGI(TAG, "Servicio de alertas iniciado (broker %s)", MQTT_BROKER_URI);
}

bool alert_service_refri_open(void)     { return s_sources[SRC_REFRI].active; }
bool alert_service_refri_has_data(void) { return s_sources[SRC_REFRI].has_data; }
bool alert_service_mqtt_connected(void) { return s_mqtt_connected; }
