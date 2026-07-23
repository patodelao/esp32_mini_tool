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
#include <stdint.h>
#include <time.h>

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

/* --- Historial -------------------------------------------------------------
 *
 * Un toast dura unos segundos: si no estabas mirando la pantalla, se perdió.
 * Toda notificación queda además registrada acá, para poder revisarla después
 * (lo muestra la tool Alertas). Solo en RAM: se vacía al reiniciar.
 */
#define NOTIFY_HIST 20

typedef struct {
    char           source[16];
    char           msg[80];
    notify_level_t level;
    time_t         ts;      /* 0 si el reloj aún no estaba en hora */
} notify_record_t;

/* Cuántas notificaciones hay guardadas (máximo NOTIFY_HIST). */
int ui_notify_history_count(void);

/* Copia la notificación i, siendo 0 la más reciente. false si i está fuera. */
bool ui_notify_history_get(int i, notify_record_t *out);

/* Vacía el historial. */
void ui_notify_history_clear(void);

#ifdef __cplusplus
}
#endif
