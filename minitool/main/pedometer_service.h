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

/* Total de pasos acumulados. */
uint32_t pedometer_steps(void);

/* Pone el contador a cero y lo persiste. */
void pedometer_reset(void);

#ifdef __cplusplus
}
#endif
