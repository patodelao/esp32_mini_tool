/*
 * dht.h — Lectura de un sensor DHT11 / DHT22 (AM2302) por un solo pin de datos.
 *
 * El ESP32-CAM es ESP-IDF (no Arduino), así que no hay librería DHT: se lee el
 * protocolo de 1 hilo a mano. Un solo pin de datos con pull-up (los módulos DHT
 * ya lo traen). Pensado para leer lento (cada varios segundos): el DHT22 admite
 * 1 lectura cada 2 s, el DHT11 cada 1 s.
 */
#pragma once

#include "driver/gpio.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DHT_TYPE_DHT11,          /* módulo azul; humedad/temp enteras */
    DHT_TYPE_DHT22,          /* AM2302, módulo blanco; con decimales y bajo cero */
} dht_type_t;

/* Configura el pin (pull-up interno de respaldo). Llamar una vez al arrancar. */
void dht_init(gpio_num_t pin);

/* Lee el sensor. Devuelve un código de estado:
 *   0     = OK (deja *temp_c en °C y *hum en %)
 *   1..4  = el sensor NO respondió (o se cortó a mitad): apunta a HARDWARE
 *           (falta pull-up 10k a 3.3V, DATA sin llegar al GPIO, alimentación)
 *   5     = llegaron los 40 bits pero el checksum no cierra: apunta al
 *           timing/ruido de la lectura (o cable largo)
 * Si el código != 0, NO se tocan los valores de salida. */
int dht_read(gpio_num_t pin, dht_type_t type, float *temp_c, float *hum);

#ifdef __cplusplus
}
#endif
