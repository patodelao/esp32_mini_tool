/*
 * ui_broker_banner.h — Aviso persistente de "broker caído".
 *
 * El broker vive en la Raspberry Pi, que el usuario apaga o reutiliza para
 * otras cosas. Cuando no está, cada nodo sigue haciendo su trabajo local (la
 * alarma del refri, los sensores, la cámara por HTTP), pero el minitool se
 * queda sin su única fuente de datos: la vista de Sensores/Nodos/Alertas deja
 * de actualizarse. Sin un aviso, eso parece que el reloj se colgó.
 *
 * Este módulo pone una píldora ámbar arriba de todo, en cualquier pantalla,
 * SOLO cuando hay Wi-Fi pero el broker no responde por un rato. Deja claro que
 * es el broker el que falta, no el reloj. Cuando la Pi vuelve, desaparece sola.
 *
 * No toca la lógica de nadie: es puramente informativo y no intercepta toques.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca el vigilante (idempotente). Llamar bajo el lock de LVGL. */
void ui_broker_banner_init(void);

#ifdef __cplusplus
}
#endif
