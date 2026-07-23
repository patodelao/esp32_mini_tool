/*
 * ble_notify.h — Notificaciones del teléfono (Android) por BLE.
 *
 * Sin app propia: se usa **Gadgetbridge** (libre, en F-Droid), que sabe
 * reenviar al reloj las notificaciones que llegan al teléfono.
 *
 * El truco es hablar el protocolo de Bangle.js, que es el más simple que
 * Gadgetbridge soporta: texto plano JSON sobre el Nordic UART Service. El
 * teléfono manda líneas de la forma
 *
 *     GB({"t":"notify","id":1,"src":"WhatsApp","title":"Ana","body":"hola"})
 *
 * y el reloj las muestra con el sistema de toasts que ya existe (así que
 * también encienden la pantalla y quedan en el historial de la tool Alertas).
 *
 * Para que Gadgetbridge lo reconozca, el nombre BLE tiene que empezar con
 * "Bangle.js" — de ahí el nombre por defecto en bt_manager.
 *
 * Aparte de las notificaciones se atiende "setTime": Gadgetbridge sincroniza
 * la hora al conectarse, lo que le da al reloj una fuente de hora que no
 * depende de internet.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Registra el servicio GATT. Debe llamarse durante el arranque de la pila
 * NimBLE, antes de lanzar la task del host. La invoca bt_manager. */
void ble_notify_register(void);

/* true si el teléfono está suscrito y llegaron mensajes. */
bool ble_notify_linked(void);

#ifdef __cplusplus
}
#endif
