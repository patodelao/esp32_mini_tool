/*
 * ui_power.c — Implementación del dormir/despertar de la pantalla.
 *
 * Detección del gesto de levantar la muñeca, con acelerómetro solamente:
 * se guarda 1 segundo de historia del eje Z (el perpendicular a la pantalla) y
 * se busca la transición "la pantalla NO miraba hacia arriba" -> "ahora sí".
 * Con el chip montado invertido, pantalla hacia arriba lee az ≈ -1g (lo mismo
 * que documenta tool_level.c).
 *
 * Pedir además un salto mínimo evita que el gesto se dispare por vibración o
 * por dejar el reloj apoyado: hace falta un movimiento real.
 */
#include "ui_power.h"
#include "bsp.h"
#include "qmi8658.h"

#include "lvgl.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "ui_power";

#define SLEEP_MS        45000   /* inactividad hasta apagar la pantalla */
#define TICK_MS           200   /* con la pantalla apagada, ritmo del IMU */
#define RAISE_HIST          5   /* muestras de historia (5 x 200 ms = 1 s) */

/* Umbrales del gesto, en g sobre el eje perpendicular a la pantalla. */
#define RAISE_FACING_UP  -0.75f  /* mirándote */
#define RAISE_WAS_AWAY   -0.45f  /* antes NO te miraba */
#define RAISE_MIN_JUMP    0.35f  /* cambio mínimo, para exigir movimiento real */

#define PWR_NS      "uipower"
#define KEY_BRIGHT  "bright"
#define KEY_RAISE   "raise"

static lv_timer_t *s_timer = NULL;
static bool  s_asleep = false;
static int   s_brightness = 100;
static bool  s_raise_wake = true;

static float s_az[RAISE_HIST];
static int   s_az_n = 0;    /* cuántas muestras válidas hay */
static int   s_az_head = 0;

/* --------------------------------- NVS ---------------------------------- */

static void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open(PWR_NS, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, KEY_BRIGHT, &v) == ESP_OK && v >= 10 && v <= 100) s_brightness = (int)v;
    if (nvs_get_i32(h, KEY_RAISE, &v) == ESP_OK) s_raise_wake = (v != 0);
    nvs_close(h);
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(PWR_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, KEY_BRIGHT, (int32_t)s_brightness);
    nvs_set_i32(h, KEY_RAISE, s_raise_wake ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

/* ------------------------------ Dormir/despertar ------------------------ */

static void screen_sleep(void)
{
    if (s_asleep) return;
    s_asleep = true;
    s_az_n = 0;                 /* la historia del gesto arranca de cero */
    bsp_backlight_set(0);
    ESP_LOGI(TAG, "Pantalla apagada");
}

void ui_power_wake(void)
{
    /* Reiniciar la inactividad también cuando ya estaba despierta: así una
     * notificación mantiene la pantalla viva un rato más. */
    lv_disp_trig_activity(NULL);
    if (!s_asleep) return;
    s_asleep = false;
    bsp_backlight_set(s_brightness);
    ESP_LOGI(TAG, "Pantalla encendida");
}

bool ui_power_asleep(void) { return s_asleep; }

/* ------------------------------ Gesto de levantar ----------------------- */

/* true si el eje Z pasó de no mirarte a mirarte dentro de la ventana. */
static bool raise_detected(float az)
{
    s_az[s_az_head] = az;
    s_az_head = (s_az_head + 1) % RAISE_HIST;
    if (s_az_n < RAISE_HIST) { s_az_n++; return false; }   /* aún llenando */

    /* La muestra más vieja de la ventana es la que acabamos de pisar. */
    float oldest = s_az[s_az_head];

    if (az > RAISE_FACING_UP) return false;        /* todavía no te mira */
    if (oldest < RAISE_WAS_AWAY) return false;     /* ya te miraba: no hay gesto */
    if (oldest - az < RAISE_MIN_JUMP) return false;/* movimiento demasiado chico */

    s_az_n = 0;   /* consumir la ventana para no re-disparar con la misma */
    return true;
}

/* --------------------------------- Tick --------------------------------- */

static void tick_cb(lv_timer_t *t)
{
    (void)t;

    uint32_t idle = lv_disp_get_inactive_time(NULL);

    if (!s_asleep) {
        if (idle > SLEEP_MS) screen_sleep();
        return;
    }

    /* Dormida: cualquier toque reinicia el contador de LVGL. */
    if (idle < SLEEP_MS) {
        ui_power_wake();
        return;
    }

    if (!s_raise_wake || !qmi8658_available()) return;

    float ax, ay, az;
    if (qmi8658_read_accel(&ax, &ay, &az) != ESP_OK) return;
    if (raise_detected(az)) {
        ESP_LOGI(TAG, "Gesto de levantar detectado (az %.2f)", (double)az);
        ui_power_wake();
    }
}

/* ------------------------------ API pública ----------------------------- */

void ui_power_init(void)
{
    if (s_timer) return;
    settings_load();
    bsp_backlight_set(s_brightness);
    s_timer = lv_timer_create(tick_cb, TICK_MS, NULL);
    ESP_LOGI(TAG, "Energía lista (apaga a los %d s, gesto %s)",
             SLEEP_MS / 1000, s_raise_wake ? "on" : "off");
}

void ui_power_set_brightness(int pct)
{
    if (pct < 10)  pct = 10;    /* por debajo de esto la pantalla no se ve */
    if (pct > 100) pct = 100;
    s_brightness = pct;
    if (!s_asleep) bsp_backlight_set(pct);
    settings_save();
}

int ui_power_get_brightness(void) { return s_brightness; }

void ui_power_set_raise_wake(bool on)
{
    s_raise_wake = on;
    settings_save();
}

bool ui_power_get_raise_wake(void) { return s_raise_wake; }
