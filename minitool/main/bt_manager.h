/*
 * bt_manager.h — Bluetooth LE (NimBLE): visibilidad del dispositivo.
 *
 * Modelo simple v1: el dispositivo puede hacerse visible (advertising BLE)
 * con un nombre configurable persistido en NVS. La pila se inicializa en
 * forma diferida la primera vez que se enciende, para no gastar RAM si no
 * se usa.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BT_NAME_MAX 29 /* límite práctico para que quepa en el advertising */

typedef enum {
    BT_STATE_OFF,         /* pila apagada o advertising detenido */
    BT_STATE_ADVERTISING, /* visible, esperando conexión */
    BT_STATE_CONNECTED,   /* alguien conectado */
    BT_STATE_UNSUPPORTED, /* firmware compilado sin soporte BT */
} bt_state_t;

/* Enciende la visibilidad BLE (init diferido de la pila si hace falta). */
void bt_manager_start(void);

/* Detiene el advertising (la pila queda cargada). */
void bt_manager_stop(void);

bt_state_t bt_manager_state(void);

/* Handle de la conexion BLE activa, o 0xFFFF si no hay. Lo usa ble_notify
 * para enviarle mensajes al telefono (musica, buscar telefono). */
uint16_t bt_manager_conn_handle(void);

/* Nombre del dispositivo (NVS, clave "bt_name"). */
void bt_manager_get_name(char *buf, size_t len);
void bt_manager_set_name(const char *name);

#ifdef __cplusplus
}
#endif
