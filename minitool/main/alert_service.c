/*
 * alert_service.c — Servicio de alertas en segundo plano.
 *
 * Se engancha al cliente MQTT compartido (mqtt_hub) con una tabla modular de
 * fuentes. Cada fuente representa un equipo (topic + textos). Al detectar una
 * transición de estado, emite una notificación flotante vía ui_notify.
 *
 * Watchdog por fuente: si el equipo publica su estado "activo" como heartbeat
 * (p.ej. el refri manda "ABIERTO" cada pocos segundos), la ausencia de mensajes
 * durante 'watchdog_ms' se interpreta como vuelta a normal (silenciosa).
 */
#include "alert_service.h"
#include "ui_notify.h"
#include "mqtt_hub.h"

#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "alert_svc";

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
    esp_timer_stop(s->wd_timer);
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
        watchdog_arm(s);
    } else {
        if (s->active) {
            s->active = false;
            if (s->msg_normal) ui_notify_push(s->name, NOTIFY_SUCCESS, s->msg_normal);
        }
        watchdog_disarm(s);
    }
}

/* Callback del hub: 'arg' es la fuente que registró este topic. */
static void alert_msg_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)topic; (void)topic_len;
    handle_message((alert_source_t *)arg, data, data_len);
}

/* --- API pública ----------------------------------------------------------- */

void alert_service_init(void)
{
    static bool s_started = false;
    if (s_started) return;
    s_started = true;

    mqtt_hub_init();
    for (size_t i = 0; i < SOURCE_COUNT; i++) {
        mqtt_hub_subscribe(s_sources[i].topic, alert_msg_cb, &s_sources[i]);
    }
    ESP_LOGI(TAG, "Servicio de alertas iniciado (%d fuente(s))", (int)SOURCE_COUNT);
}

bool alert_service_refri_open(void)     { return s_sources[SRC_REFRI].active; }
bool alert_service_refri_has_data(void) { return s_sources[SRC_REFRI].has_data; }
bool alert_service_mqtt_connected(void) { return mqtt_hub_connected(); }
