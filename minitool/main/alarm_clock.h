/*
 * alarm_clock.h — Alarmas diarias (despertador).
 *
 * Es la función de reloj que faltaba. El temporizador de la tool Reloj sirve
 * para "avisame en 10 minutos"; esto es lo otro: "todos los días a las 7:30".
 *
 * El servicio corre SIEMPRE, como el podómetro: la alarma tiene que sonar esté
 * abierta la tool que esté, o con la pantalla apagada. Por eso el chequeo vive
 * acá y no en la herramienta, que es solo la vista para editarlas.
 *
 * Se apoya en lo que ya existe:
 *   - ui_power para encender la pantalla (una alarma visual a oscuras no sirve),
 *   - ui_notify para que quede registrada en el historial,
 *   - NVS para que sobrevivan a un corte de luz.
 *
 * No hay zumbador en esta placa, así que el aviso es visual: una pantalla
 * parpadeante que se cierra al tocarla.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALARM_COUNT 4   /* alarmas configurables */

typedef struct {
    uint8_t hour;    /* 0..23 */
    uint8_t min;     /* 0..59 */
    bool    enabled;
} alarm_t;

/* Arranca el vigilante. Llamar una vez con LVGL listo y bajo su lock. */
void alarm_clock_init(void);

/* Lee/escribe la alarma i (0..ALARM_COUNT-1). set() persiste en NVS. */
bool alarm_clock_get(int i, alarm_t *out);
void alarm_clock_set(int i, const alarm_t *a);

/* Cuántas hay activadas, para mostrarlo de un vistazo. */
int alarm_clock_enabled_count(void);

/* Silencia la alarma que esté sonando. */
void alarm_clock_dismiss(void);

#ifdef __cplusplus
}
#endif
