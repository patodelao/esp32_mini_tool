/*
 * qmi8658.h — Driver mínimo del IMU QMI8658 (acelerómetro + giroscopio, 6 ejes)
 * integrado en la Waveshare ESP32-S3-Touch-LCD-1.28.
 *
 * Comparte el bus I2C del touch (BSP_I2C_NUM); bsp_init() debe haberse
 * ejecutado antes de qmi8658_init(). Las lecturas se hacen desde lv_timers
 * (task de LVGL), igual que el touch, así que no hay acceso concurrente al bus.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Detecta e inicializa el IMU (accel ±4g, gyro ±256dps, 125 Hz). */
esp_err_t qmi8658_init(void);

/* true si el init encontró el chip y respondió correctamente. */
bool qmi8658_available(void);

/* Aceleración en g por eje. */
esp_err_t qmi8658_read_accel(float *ax, float *ay, float *az);

/* Velocidad angular en grados/segundo por eje. */
esp_err_t qmi8658_read_gyro(float *gx, float *gy, float *gz);

#ifdef __cplusplus
}
#endif
