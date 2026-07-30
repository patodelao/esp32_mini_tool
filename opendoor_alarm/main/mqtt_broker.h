/*
 * mqtt_broker.h — Broker MQTT embebido del home-lab, hospedado en el nodo del
 * refri (opendoor).
 *
 * Antes toda la flota dependía del broker Mosquitto de la Raspberry Pi. La Pi se
 * apaga o se reutiliza, y sin ella nadie podía publicar (se cortaba la captura) ni
 * el minitool mostrar datos (se cortaba la visualización). Este nodo, en cambio,
 * está siempre enchufado vigilando la puerta: es el candidato natural para
 * hospedar el broker y que la red no dependa de la Pi.
 *
 * Corre sobre Mongoose (sockets lwIP) en su propia task, escuchando en el 1883.
 * Implementa lo que la flota realmente usa: CONNECT/CONNACK anónimo,
 * SUBSCRIBE/SUBACK, PUBLISH/PUBACK, PINGREQ/PINGRESP, fanout con wildcards
 * (+/#) y mensajes retenidos (para que el minitool repueble su vista al conectar).
 *
 * El propio refri publica su telemetría/puerta y escucha sus comandos SIN abrir
 * un cliente TCP a sí mismo: usa la API in-process de abajo, que inyecta y recibe
 * mensajes directo en el broker. Así no depende de que lwIP tenga loopback
 * compilado ni gasta un socket extra.
 *
 * Limitaciones asumidas (ver README):
 *   - Los retenidos viven en RAM: un reboot/OTA del refri los pierde hasta que
 *     cada nodo republica. Si la Pi está de bridge, se los devuelve al instante.
 *   - Sin Last-Will: si un nodo cae de golpe, el broker no publica su "offline".
 *     No importa para el minitool, que da por caído a un nodo por ANTIGÜEDAD de
 *     sus datos (~180 s), no por el will. Solo tarda un poco más en notarlo.
 *   - Anónimo (igual que el allow_anonymous actual de Mosquitto).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callback de un suscriptor LOCAL (in-process). topic y payload llegan
 * terminados en '\0'; payload_len es por si el payload trae ceros. */
typedef void (*mqtt_broker_local_cb_t)(const char *topic, const char *payload,
                                       int payload_len);

/* Registra un suscriptor local. Llamar ANTES de mqtt_broker_start() (se lee solo
 * desde la task del broker; registrarlo antes evita cualquier carrera). El filtro
 * admite comodines MQTT (+/#), p.ej. "labo/nodo/refri/cmd". */
void mqtt_broker_on_local(const char *filter, mqtt_broker_local_cb_t cb);

/* Arranca la task del broker (idempotente). Llamar con la red ya arriba. */
void mqtt_broker_start(void);

/* true una vez que el broker está escuchando. */
bool mqtt_broker_running(void);

/* Telemetría del broker para publicar como sensores (tool Sensores):
 * client_count = conexiones TCP vivas ahora; reaped = zombies cerrados en total. */
int      mqtt_broker_client_count(void);
uint32_t mqtt_broker_reaped(void);

/* Publica un mensaje en el broker desde este nodo, como si llegara de un cliente:
 * actualiza el retenido (si retain) y lo reparte a los suscriptores que
 * coincidan. Seguro de llamar desde cualquier task (encola; el reparto ocurre en
 * la task del broker). payload NULL o "" con retain=true borra el retenido. */
void mqtt_broker_local_publish(const char *topic, const char *payload, bool retain);

#ifdef __cplusplus
}
#endif
