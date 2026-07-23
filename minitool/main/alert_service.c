/*
 * alert_service.c — Servicio de alertas en segundo plano.
 *
 * Dos responsabilidades, sobre el cliente MQTT compartido (mqtt_hub):
 *
 *  1) BUS DE ALERTAS MULTICANAL (`labo/alerta/#`): cualquier equipo publica un
 *     JSON {"origen","nivel","msg"} y se muestra como notificación flotante.
 *     Es agnóstico del equipo: sumar uno nuevo no requiere tocar el minitool.
 *
 *  2) ESTADO DEL REFRI (`labo/nodo/refri/puerta`): mantiene abierto/cerrado
 *     para la vista Dashboard (con watchdog). Ya NO emite notificación por aquí:
 *     eso lo hace el bus (niveles aviso/alarma/ok), evitando duplicados.
 *
 *     Va bajo labo/nodo/<id>/ y no bajo labo/sensor/<id>/ porque el payload es
 *     texto, no un número: los sensores del minitool asumen números.
 */
#include "alert_service.h"
#include "ui_notify.h"
#include "mqtt_hub.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "cJSON.h"

#include <string.h>

static const char *TAG = "alert_svc";

#define ALERT_BUS_FILTER "labo/alerta/#"

/* --- Bus de alertas multicanal -------------------------------------------- */

static notify_level_t level_from_str(const char *s)
{
    if (!s)                       return NOTIFY_INFO;
    if (strcmp(s, "alarma") == 0) return NOTIFY_ALERT;
    if (strcmp(s, "aviso")  == 0) return NOTIFY_WARNING;
    if (strcmp(s, "ok")     == 0) return NOTIFY_SUCCESS;
    return NOTIFY_INFO;
}

static void alert_bus_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)topic; (void)topic_len; (void)arg;

    char buf[192];
    int n = data_len < (int)sizeof(buf) - 1 ? data_len : (int)sizeof(buf) - 1;
    memcpy(buf, data, n);
    buf[n] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "Alerta con JSON inválido: %s", buf);
        return;
    }
    cJSON *jorigen = cJSON_GetObjectItem(root, "origen");
    cJSON *jnivel  = cJSON_GetObjectItem(root, "nivel");
    cJSON *jmsg    = cJSON_GetObjectItem(root, "msg");

    const char *origen = (cJSON_IsString(jorigen)) ? jorigen->valuestring : "Alerta";
    const char *nivel  = (cJSON_IsString(jnivel))  ? jnivel->valuestring  : NULL;
    const char *msg    = (cJSON_IsString(jmsg))    ? jmsg->valuestring    : "";

    ui_notify_push(origen, level_from_str(nivel), msg);
    cJSON_Delete(root);
}

/* --- Estado del refri (para el Dashboard) --------------------------------- */

typedef struct {
    const char *topic;        /* topic MQTT del estado                      */
    const char *active_word;  /* payload que significa "activo/abierto"     */
    uint32_t    watchdog_ms;  /* 0 = sin watchdog                           */

    bool               active;
    bool               has_data;
    esp_timer_handle_t wd_timer;
} door_source_t;

static door_source_t s_refri = {
    .topic       = "labo/nodo/refri/puerta",
    .active_word = "ABIERTO",
    .watchdog_ms = 12000,
};

/* Topic anterior, de cuando el refri no seguía la convención labo/. Se sigue
 * escuchando porque ese nodo no tiene OTA: hasta que lo flashees por cable
 * sigue publicando acá, y sin esto el Dashboard quedaría ciego. Se puede
 * borrar esta línea una vez que el refri esté actualizado. */
#define REFRI_TOPIC_LEGACY "proyectos/casa/refri/puerta"

static void watchdog_cb(void *arg)
{
    door_source_t *s = (door_source_t *)arg;
    if (s->active) {
        ESP_LOGW(TAG, "watchdog: sin heartbeat, se asume cerrado");
        s->active = false; /* sin señal por un rato => cerrado (silencioso) */
    }
}

static void watchdog_arm(door_source_t *s)
{
    if (!s->watchdog_ms) return;
    if (!s->wd_timer) {
        const esp_timer_create_args_t args = { .callback = watchdog_cb, .arg = s, .name = "alert_wd" };
        if (esp_timer_create(&args, &s->wd_timer) != ESP_OK) return;
    }
    esp_timer_stop(s->wd_timer);
    esp_timer_start_once(s->wd_timer, (uint64_t)s->watchdog_ms * 1000ULL);
}

static void watchdog_disarm(door_source_t *s)
{
    if (s->wd_timer) esp_timer_stop(s->wd_timer);
}

/* Solo actualiza estado (abierto/cerrado). La notificación la da el bus. */
static void door_cb(const char *topic, int topic_len, const char *data, int data_len, void *arg)
{
    (void)topic; (void)topic_len;
    door_source_t *s = (door_source_t *)arg;
    s->has_data = true;

    bool now_active = ((int)strlen(s->active_word) == data_len &&
                       strncmp(data, s->active_word, data_len) == 0);
    if (now_active) {
        s->active = true;
        watchdog_arm(s);
    } else {
        s->active = false;
        watchdog_disarm(s);
    }
}

/* --- API pública ----------------------------------------------------------- */

void alert_service_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;

    mqtt_hub_init();
    mqtt_hub_subscribe(ALERT_BUS_FILTER, alert_bus_cb, NULL);       /* multicanal */
    mqtt_hub_subscribe(s_refri.topic, door_cb, &s_refri);           /* estado refri */
    mqtt_hub_subscribe(REFRI_TOPIC_LEGACY, door_cb, &s_refri);      /* topic viejo */
    ESP_LOGI(TAG, "Alertas: bus %s + estado %s (y %s)",
             ALERT_BUS_FILTER, s_refri.topic, REFRI_TOPIC_LEGACY);
}

bool alert_service_refri_open(void)     { return s_refri.active; }
bool alert_service_refri_has_data(void) { return s_refri.has_data; }
bool alert_service_mqtt_connected(void) { return mqtt_hub_connected(); }
