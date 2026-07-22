/*
 * weather_service.h — Servicio de clima con caché compartida.
 *
 * Centraliza la descarga del clima (ubicación por IP + Open-Meteo) para que
 * varios consumidores (la tool "Clima" y el screensaver) compartan el mismo
 * dato sin repetir descargas. La descarga es asíncrona (tarea de FreeRTOS) y
 * el resultado se guarda en una caché protegida por mutex.
 *
 * Los consumidores leen con weather_service_get() y detectan cambios con
 * weather_service_generation() (se incrementa en cada actualización exitosa).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char city[32];   /* ciudad                         */
    char temp[16];   /* temperatura ya formateada "x °C" */
    char emoji[16];  /* emoji del clima (font_weather)  */
    char desc[32];   /* descripción corta               */
    bool valid;      /* true si hay al menos un dato bueno */
} weather_data_t;

/* Inicializa el servicio (crea el mutex). Idempotente. */
void weather_service_init(void);

/* Lanza una descarga asíncrona si hay Wi-Fi y no hay otra en curso.
 * force=false respeta el tiempo mínimo entre descargas (caché fresca);
 * force=true descarga siempre (p.ej. botón "Refrescar"). */
void weather_service_refresh(bool force);

/* Copia la caché actual a 'out'. Devuelve out->valid. */
bool weather_service_get(weather_data_t *out);

/* Contador que se incrementa en cada actualización exitosa (para pollers). */
uint32_t weather_service_generation(void);

/* true si hay una descarga en curso. */
bool weather_service_is_fetching(void);

#ifdef __cplusplus
}
#endif
