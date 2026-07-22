/*
 * sensor_service.h — Lecturas de sensores vía MQTT con histórico corto.
 *
 * Se suscribe a "labo/sensor/#". El id del sensor es lo que sigue tras
 * "labo/sensor/" (p.ej. "sala/temp"). El payload se toma como número
 * (p.ej. "23.5"); se guarda el último valor y un histórico para graficar.
 *
 * Convención de topic (ajústala en SENSOR_FILTER/PREFIX si usas otra):
 *   labo/sensor/<id>  ->  "<numero>"
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_HIST 30

void sensor_service_init(void);

/* Cantidad de sensores vistos. */
int sensor_count(void);

/* Datos del sensor i (id, valor formateado, antigüedad en s). false si i fuera de rango. */
bool sensor_get(int i, char *id, int id_size, char *val, int val_size, uint32_t *age_s);

/* Copia el histórico del sensor i (más viejo primero) en 'out'. Devuelve nº de puntos. */
int sensor_history(int i, float *out, int max);

#ifdef __cplusplus
}
#endif
