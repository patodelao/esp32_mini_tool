/*
 * ui_power.h — Gestión de la pantalla: dormirla y despertarla.
 *
 * Hasta ahora el backlight quedaba encendido para siempre: había screensaver
 * (el watchface) pero la pantalla nunca se apagaba, que es de donde sale casi
 * todo el consumo de la placa.
 *
 * Comportamiento, al estilo de un reloj:
 *   - sin tocar nada, a los 20 s aparece el watchface (eso ya lo hacía ui_menu),
 *   - a los SLEEP_MS se apaga el backlight,
 *   - se despierta al tocar la pantalla, al levantar la muñeca (gesto detectado
 *     con el acelerómetro) o cuando entra una notificación.
 *
 * Todo corre en un lv_timer, o sea en el hilo de LVGL: el IMU se lee desde ahí
 * igual que el touch, así que no hay acceso concurrente al bus I2C.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca la gestión de energía. Llamar una vez, con LVGL ya inicializado
 * (después de bsp_init) y bajo el lock. Idempotente. */
void ui_power_init(void);

/* Despierta la pantalla y reinicia el contador de inactividad. Seguro de
 * llamar desde el hilo de LVGL (lo usa ui_notify al mostrar un toast). */
void ui_power_wake(void);

/* true si la pantalla está apagada. */
bool ui_power_asleep(void);

/* Brillo con el que se enciende la pantalla (0..100). Se guarda en NVS. */
void ui_power_set_brightness(int pct);
int  ui_power_get_brightness(void);

/* Despertar por gesto (levantar la muñeca). Se guarda en NVS. */
void ui_power_set_raise_wake(bool on);
bool ui_power_get_raise_wake(void);

#ifdef __cplusplus
}
#endif
