/*
 * pedometer_service.h — Contador de pasos en segundo plano.
 *
 * Detecta pasos a partir de la magnitud del acelerómetro. El muestreo corre en
 * un lv_timer global (hilo de LVGL, igual que el touch/IMU), de modo que cuenta
 * SIEMPRE, sin importar qué herramienta esté abierta y sin acceso concurrente
 * al bus I2C. El total se persiste en NVS y sobrevive reinicios.
 *
 * La tool "Pasos" es solo una vista que lee pedometer_steps().
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca el contador (crea el lv_timer y carga el total de NVS).
 * Debe llamarse con el lock de LVGL tomado. Idempotente. */
void pedometer_service_init(void);

/* Pasos de HOY. El contador se reinicia solo al cambiar el día (medianoche
 * local), como en cualquier reloj: un total que crece para siempre no dice
 * nada sobre si te moviste. */
uint32_t pedometer_steps(void);

/* Pone el contador de hoy a cero y lo persiste. No toca el historial. */
void pedometer_reset(void);

/* --- Historial ------------------------------------------------------------ */

#define PEDOMETER_DAYS 7   /* días guardados, sin contar el de hoy */

/* Copia los últimos días cerrados en 'out' (el más viejo primero) y devuelve
 * cuántos escribió. Se persiste en NVS. */
int pedometer_history(uint32_t *out, int max);

/* Meta diaria de pasos (persistida). 0 = sin meta. */
void     pedometer_set_goal(uint32_t steps);
uint32_t pedometer_get_goal(void);

#ifdef __cplusplus
}
#endif
