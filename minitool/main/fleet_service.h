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

/* true si ese nodo se declara online. Sirve para no confundir "se cayó" con
 * "está durmiendo": un nodo de bajo consumo (el refri) publica offline limpio
 * antes de dormirse, así que su silencio posterior es esperable. */
bool fleet_is_online(const char *id);

/* IP del nodo i, si la publicó. Devuelve false si no se conoce. */
bool fleet_get_ip(int i, char *ip, int ip_size);

/* Marca el estado de un nodo SIN pasar por el broker.
 *
 * Existe por el propio reloj: publicaba su "online" y se quedaba esperando que
 * el broker se lo devolviera para enterarse. Ese eco no siempre vuelve, así
 * que el minitool se mostraba offline a sí mismo estando conectado y
 * publicando. Preguntarle a la red por un estado que uno ya conoce es frágil
 * y, encima, más lento. */
void fleet_set_local(const char *id, bool online);

/* Refresca la "última evidencia de vida" de un nodo sin tocar su flag online.
 * sensor_service la llama con el id del nodo en cada lectura recibida, para que
 * un nodo que sigue publicando no aparezca "sin señal" por un offline espurio. */
void fleet_note_activity(const char *id);

#ifdef __cplusplus
}
#endif
