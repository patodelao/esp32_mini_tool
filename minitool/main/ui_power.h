/*
 * ui_power.h — Gestión de la pantalla: dormirla y despertarla.
 *
 * Hasta ahora el backlight quedaba encendido para siempre: había screensaver
 * (el watchface) pero la pantalla nunca se apagaba, que es de donde sale casi
 * todo el consumo de la placa.
 *
 * Comportamiento:
 *   - sin tocar nada, a los 20 s aparece el watchface (eso lo hace ui_menu),
 *   - al cumplirse el tiempo configurado se apaga el backlight,
 *   - se despierta al tocar la pantalla, al moverlo (cualquier manipulación) o
 *     cuando entra una notificación.
 *
 * Todo corre en un lv_timer, o sea en el hilo de LVGL: el IMU se lee desde ahí
 * igual que el touch, así que no hay acceso concurrente al bus I2C.
 *
 * Los ajustes se editan desde la tool Config y se guardan en NVS.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Qué tan poco movimiento hace falta para despertarlo. */
typedef enum {
    UI_POWER_SENS_BAJA = 0,   /* hay que levantarlo o girarlo con ganas   */
    UI_POWER_SENS_MEDIA,      /* moverlo sobre la mesa alcanza            */
    UI_POWER_SENS_ALTA,       /* un roce; puede despertarlo la vibración  */
    UI_POWER_SENS_COUNT,
} ui_power_sens_t;

/* Arranca la gestión de energía. Llamar una vez, con LVGL ya inicializado
 * (después de bsp_init) y bajo el lock. Idempotente. */
void ui_power_init(void);

/* Despierta la pantalla y reinicia el contador de inactividad. Seguro de
 * llamar desde el hilo de LVGL (lo usa ui_notify al mostrar un toast). */
void ui_power_wake(void);

/* true si la pantalla está apagada. */
bool ui_power_asleep(void);

/* Impide que la pantalla se duerma mientras dure algo que se mira sin tocar.
 * La linterna es el caso claro: se apagaría sola justo cuando la estás usando.
 * Al activarlo despierta la pantalla; al soltarlo, el conteo de inactividad
 * arranca de nuevo. Llamar siempre en pares (open/close de la tool). */
void ui_power_inhibit(bool on);

/* Brillo con el que se enciende la pantalla (10..100). */
void ui_power_set_brightness(int pct);
int  ui_power_get_brightness(void);

/* Despertar al mover. */
void ui_power_set_motion_wake(bool on);
bool ui_power_get_motion_wake(void);

/* Sensibilidad del despertar por movimiento. */
void ui_power_set_sensitivity(ui_power_sens_t s);
ui_power_sens_t ui_power_get_sensitivity(void);
const char *ui_power_sens_name(ui_power_sens_t s);

/* Segundos de inactividad hasta apagar la pantalla. 0 = no apagar nunca. */
void ui_power_set_sleep_s(int seconds);
int  ui_power_get_sleep_s(void);

/* --- Modo noche ------------------------------------------------------------
 *
 * En la franja configurada el reloj no interrumpe: las notificaciones NO se
 * muestran ni encienden la pantalla (siguen quedando en el historial de la
 * tool Alertas), y si la despertás a mano lo hace con brillo reducido.
 *
 * Las alertas de nivel crítico (NOTIFY_ALERT) sí pasan: para eso están.
 */
void ui_power_set_night(bool on);
bool ui_power_get_night(void);

/* Franja, en horas locales (0..23). Puede cruzar la medianoche (22 -> 7). */
void ui_power_set_night_range(int start_h, int end_h);
int  ui_power_get_night_start(void);
int  ui_power_get_night_end(void);

/* true si el modo noche está activo Y estamos dentro de la franja ahora.
 * false si el reloj todavía no tiene la hora en hora. */
bool ui_power_night_now(void);

#ifdef __cplusplus
}
#endif
