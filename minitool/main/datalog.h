/*
 * datalog.h — Registro histórico de sensores en la flash, descargable en CSV.
 *
 * Lo que el sistema NO tenía: memoria larga. En NVS hay 24 h de promedios por
 * hora y 7 días de récords min/max, que alcanzan para mirar la pantalla pero no
 * para responder "¿cómo se secó la maceta el mes pasado?" ni para llevarse los
 * datos a una planilla.
 *
 * Cada media hora se escribe una foto de todos los sensores vivos en la
 * partición `storage` (1 MB de SPIFFS que hasta ahora estaba declarada y sin
 * usar). El formato es CSV plano, para abrirlo con doble clic:
 *
 *     fecha,sensor,valor
 *     2026-07-24 15:30,pieza/temp,18.9
 *
 * Se descarga entero desde el panel web:  http://<ip>/csv
 *
 * ROTACIÓN: dos archivos. Cuando el actual pasa el límite, el anterior se
 * borra, el actual pasa a ser el anterior y se empieza uno nuevo. Así el
 * registro nunca llena la partición y siempre quedan entre uno y dos límites de
 * historia — unas 3 a 6 semanas. No se copia nada: en una partición de 1 MB no
 * hay lugar para duplicar el archivo mientras se lo recorta.
 *
 * Si la partición no monta, el módulo se desactiva solo y el resto del reloj
 * sigue igual.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Monta SPIFFS y arranca la tarea de registro. Idempotente. */
void datalog_init(void);

/* false si la partición no montó: no hay registro que mostrar ni descargar. */
bool datalog_ready(void);

/* Bytes ocupados por el registro (los dos archivos). */
size_t datalog_size(void);

/* Vuelca el registro completo en orden cronológico, con el encabezado CSV
 * adelante. 'sink' recibe trozos de texto; si devuelve false se corta (el
 * cliente HTTP cerró). Devuelve false si no hay nada que volcar. */
typedef bool (*datalog_sink_t)(void *ctx, const char *data, int len);
bool datalog_dump(datalog_sink_t sink, void *ctx);

/* Borra el registro y empieza de cero. */
void datalog_clear(void);

#ifdef __cplusplus
}
#endif
