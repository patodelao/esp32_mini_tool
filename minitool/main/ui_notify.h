/*
 * ui_notify.h — Sistema de notificaciones flotantes (toasts) global.
 *
 * Diseñado para ser modular y agnóstico del productor: cualquier equipo o
 * subsistema (sensor del refri, y en el futuro otros dispositivos) puede
 * emitir una alerta con ui_notify_push() desde CUALQUIER tarea de FreeRTOS,
 * sin tomar el lock de LVGL ni conocer detalles de la UI.
 *
 * Internamente, ui_notify_push() encola la notificación en un buffer
 * thread-safe. Un temporizador de LVGL (en el hilo gráfico) drena la cola y
 * dibuja el toast sobre todo lo demás (lv_layer_top), de modo que la alerta
 * aparece aunque estés dentro de otra herramienta o en el screensaver.
 *
 * Uso:
 *   ui_notify_init();                                  // una vez, tras bsp_init
 *   ui_notify_push("Refri", NOTIFY_ALERT, "Puerta abierta");
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NOTIFY_INFO = 0,   /* informativo (azul)   */
    NOTIFY_SUCCESS,    /* ok / resuelto (verde) */
    NOTIFY_WARNING,    /* advertencia (ámbar)   */
    NOTIFY_ALERT,      /* alerta crítica (rojo) */
} notify_level_t;

/* Inicializa el sistema. Debe llamarse una vez, con LVGL ya inicializado
 * (después de bsp_init). Idempotente. */
void ui_notify_init(void);

/* Emite una notificación flotante. Seguro de llamar desde cualquier tarea,
 * con o sin el lock de LVGL tomado. 'source' es el nombre del equipo/origen
 * (p.ej. "Refri"); 'msg' el texto a mostrar. Ambos se copian. */
void ui_notify_push(const char *source, notify_level_t level, const char *msg);

#ifdef __cplusplus
}
#endif
