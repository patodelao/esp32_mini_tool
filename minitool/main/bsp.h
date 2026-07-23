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
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Resolución de la pantalla redonda */
#define BSP_LCD_H_RES 240
#define BSP_LCD_V_RES 240

/* Puerto e I2C compartido (touch CST816S + IMU QMI8658) */
#define BSP_I2C_NUM   0
#define BSP_I2C_SDA   6
#define BSP_I2C_SCL   7
#define BSP_I2C_HZ    400000

/* Bus I2C maestro compartido. Cada driver se cuelga con
 * i2c_master_bus_add_device(); el bus serializa los accesos por su cuenta.
 * Válido recién después de bsp_init(). */
i2c_master_bus_handle_t bsp_i2c_bus(void);

/*
 * Inicializa todo el hardware y LVGL, y arranca la task de renderizado.
 * Tras llamar a esta función se puede construir la UI (protegida por el lock).
 */
void bsp_init(void);

/* Handle del controlador táctil (por si alguna tool lo necesita). */
esp_lcd_touch_handle_t bsp_touch_handle(void);

/* Brillo del backlight, 0..100 (%). 0 apaga la pantalla sin apagar el panel,
 * que es lo que usa ui_power para dormirla cuando no la estás mirando.
 * Está en PWM, así que admite valores intermedios (modo noche). */
void bsp_backlight_set(int pct);

/* Brillo actual. */
int bsp_backlight_get(void);

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
