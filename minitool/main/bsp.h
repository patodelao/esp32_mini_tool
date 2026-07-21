/*
 * bsp.h — Board Support Package para Waveshare ESP32-S3-Touch-LCD-1.28
 *
 * Encapsula la inicialización del hardware (SPI + LCD GC9A01, I2C + touch
 * CST816S) y del port de LVGL (buffers, driver de display, driver de entrada,
 * tick timer y task de renderizado con su mutex).
 */
#pragma once

#include "lvgl.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Resolución de la pantalla redonda */
#define BSP_LCD_H_RES 240
#define BSP_LCD_V_RES 240

/* Puerto e I2C compartido (touch CST816S + IMU QMI8658) */
#define BSP_I2C_NUM   I2C_NUM_0
#define BSP_I2C_SDA   6
#define BSP_I2C_SCL   7

/*
 * Inicializa todo el hardware y LVGL, y arranca la task de renderizado.
 * Tras llamar a esta función se puede construir la UI (protegida por el lock).
 */
void bsp_init(void);

/* Handle del controlador táctil (por si alguna tool lo necesita). */
esp_lcd_touch_handle_t bsp_touch_handle(void);

/*
 * Lock/unlock del mutex de LVGL. LVGL NO es thread-safe: cualquier acceso a la
 * API de LVGL desde una task distinta a la de renderizado debe ir entre
 * bsp_lvgl_lock()/bsp_lvgl_unlock(). timeout_ms = -1 bloquea indefinidamente.
 */
bool bsp_lvgl_lock(int timeout_ms);
void bsp_lvgl_unlock(void);

#ifdef __cplusplus
}
#endif
