/*
 * sensor_service.h — Lecturas de sensores vía MQTT con histórico corto.
 *
 * Se suscribe a "labo/sensor/#". El id del sensor es lo que sigue tras
 * "labo/sensor/" (p.ej. "pieza/temp"). El payload se toma como número
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

#define SENSOR_HIST   30   /* puntos del histórico corto (uno por lectura) */
#define SENSOR_HIST_H 24   /* horas del histórico largo (uno por hora) */
#define SENSOR_ID_MAX 32

void sensor_service_init(void);

/* Cantidad de sensores vistos. */
int sensor_count(void);

/* Datos del sensor i (id, valor formateado, antigüedad en s). false si i fuera de rango. */
bool sensor_get(int i, char *id, int id_size, char *val, int val_size, uint32_t *age_s);

/* Copia el histórico del sensor i (más viejo primero) en 'out'. Devuelve nº de puntos. */
int sensor_history(int i, float *out, int max);

/* Histórico largo: un promedio por hora, hasta SENSOR_HIST_H horas (más viejo
 * primero). Sirve para ver la curva de un día entero — p.ej. cuánto tarda en
 * secarse la maceta — y elegir el umbral de riego con datos en vez de a ojo.
 * Se persiste en NVS, así sobrevive a reinicios. Devuelve nº de puntos. */
int sensor_history_hourly(int i, float *out, int max);

/* Récord diario del sensor i: min y max acumulados (reinicio a medianoche o
 * manual). *valid queda false si aún no hay datos. false si i fuera de rango. */
bool sensor_get_record(int i, float *min, float *max, bool *valid);

/* Borra el récord del sensor i y lo reinicia desde su último valor. */
void sensor_reset_record(int i);

/* Olvida el sensor i: libera su lugar y borra su récord, su histórico y su
 * regla de alertas. Además borra el mensaje retenido de su topic en el broker
 * (payload vacío), porque si no reaparecería en la próxima reconexión.
 *
 * Es para limpiar sensores que ya no existen: el "Crudo" que deja una sesión
 * de calibración, o los fantasmas de un nodo que renombraste. */
void sensor_forget(int i);

/* --- Nombres y unidades (compartidos por la UI y el motor de alertas) ------ */

/* Magnitud del id: lo que sigue al último '/' ("temp", "suelo", ...). */
const char *sensor_leaf(const char *id);

/* Unidad inferida de la magnitud ("\xC2\xB0C", "%", "dBm", "s" o ""). */
const char *sensor_unit(const char *id);

/* Nombre legible: "pieza/temp" -> "Temp Pieza". ASCII: la fuente del display
 * no tiene acentos ni ñ. */
void sensor_friendly_name(const char *id, char *out, int out_size);

/* Nodo al que pertenece el sensor: "pieza/temp" -> "pieza". */
void sensor_node_id(const char *id, char *out, int out_size);

/* Nombre legible del nodo: "pieza" -> "Pieza". */
const char *sensor_node_label(const char *node);

#ifdef __cplusplus
}
#endif
