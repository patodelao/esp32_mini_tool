/*
 * alert_service.h — Servicio de alertas en segundo plano (MQTT).
 *
 * Corre independiente de la UI: mantiene un cliente MQTT persistente y una
 * TABLA MODULAR DE FUENTES (equipos). Cuando una fuente cambia de estado,
 * emite una notificación flotante vía ui_notify. Así las alertas aparecen
 * aunque estés en otra herramienta o en el screensaver.
 *
 * Para añadir un equipo nuevo en el futuro basta con agregar una entrada a la
 * tabla s_sources[] en alert_service.c (topic, nombre y textos). No hay que
 * tocar la UI ni el ciclo MQTT.
 *
 * El dashboard consume el estado con los getters de conveniencia en vez de
 * abrir su propio cliente MQTT.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca el cliente MQTT en segundo plano y se suscribe a todas las fuentes.
 * Debe llamarse una vez, tras wifi_manager_init y ui_notify_init. Idempotente.
 * La conexión real ocurre cuando haya Wi-Fi (esp-mqtt reintenta solo). */
void alert_service_init(void);

/* --- Getters de conveniencia para el dashboard (fuente "Refri") ----------- */

/* true si la puerta del refri está actualmente abierta. */
bool alert_service_refri_open(void);

/* true si alguna vez se recibió dato del refri en esta sesión. */
bool alert_service_refri_has_data(void);

/* true si el cliente MQTT está conectado al broker. */
bool alert_service_mqtt_connected(void);

#ifdef __cplusplus
}
#endif
