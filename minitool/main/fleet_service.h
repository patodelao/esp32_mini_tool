/*
 * fleet_service.h — Registro de nodos (ESP/equipos) vía MQTT.
 *
 * Se suscribe a "labo/nodo/+/status" en el hub. Cada nodo publica en
 * "labo/nodo/<id>/status" el payload "online" u "offline" (idealmente con
 * retain y un last-will "offline"). El servicio arma dinámicamente la lista de
 * nodos vistos y su último estado; la tool Fleet solo la muestra.
 *
 * Convención de topic (ajústala en FLEET_FILTER si usas otra):
 *   labo/nodo/<id>/status  ->  "online" | "offline"
 *   labo/nodo/<id>/ip      ->  "192.168.0.42"   (opcional, retenido)
 *
 * La IP es opcional pero práctica: es la que se usa para actualizar el nodo
 * por OTA cuando el nombre mDNS (<id>.local) no resuelve.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void fleet_service_init(void);

/* Cantidad de nodos vistos en esta sesión. */
int fleet_count(void);

/* Datos del nodo i. Devuelve false si i está fuera de rango.
 * 'online' = último estado reportado; 'age_s' = segundos desde el último
 * mensaje (útil para marcar "sin señal" si es muy alto). */
bool fleet_get(int i, char *id, int id_size, bool *online, uint32_t *age_s);

/* IP del nodo i, si la publicó. Devuelve false si no se conoce. */
bool fleet_get_ip(int i, char *ip, int ip_size);

#ifdef __cplusplus
}
#endif
