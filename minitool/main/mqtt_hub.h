/*
 * mqtt_hub.h — Cliente MQTT único y compartido para todo el minitool.
 *
 * Un solo cliente al broker; varios subsistemas (alertas, fleet, sensores,
 * control) se enganchan con mqtt_hub_subscribe() indicando un filtro de topic
 * (admite comodines MQTT '+' y '#') y un callback. Para enviar comandos se usa
 * mqtt_hub_publish(). Los callbacks corren en el contexto de la tarea MQTT: no
 * toques LVGL directamente desde ellos (usa ui_notify o variables de estado).
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* topic/data NO están terminados en '\0': usa las longitudes. */
typedef void (*mqtt_hub_cb_t)(const char *topic, int topic_len,
                              const char *data, int data_len, void *arg);

/* Inicia el cliente (idempotente). Se conecta cuando haya Wi-Fi. */
void mqtt_hub_init(void);

/* true si está conectado al broker. */
bool mqtt_hub_connected(void);

/* Registra un filtro de suscripción y su callback. Si ya hay conexión,
 * se suscribe de inmediato; si no, al conectar. 'arg' se pasa al callback. */
void mqtt_hub_subscribe(const char *filter, mqtt_hub_cb_t cb, void *arg);

/* Publica un mensaje. payload se toma como cadena (len automática).
 * Devuelve el message_id de esp-mqtt (>=0) o -1 si no hay cliente. */
int mqtt_hub_publish(const char *topic, const char *payload, int qos, bool retain);

#ifdef __cplusplus
}
#endif
