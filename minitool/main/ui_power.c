/*
 * ui_power.c — Implementación del dormir/despertar de la pantalla.
 *
 * Despertar por movimiento: en vez de buscar el gesto específico de levantar
 * la muñeca (que exige un movimiento amplio y bien orientado), se despierta
 * ante CUALQUIER manipulación. El reloj está sobre el escritorio y sin
 * batería, así que lo que se busca es que se encienda al agarrarlo, no ahorrar
 * energía a costa de la respuesta.
 *
 * Se miran dos señales y alcanza con una:
 *   - el giroscopio, que delata cualquier rotación por chica que sea;
 *   - el cambio del vector de gravedad entre lecturas, que capta inclinaciones
 *     y desplazamientos.
 *
 * La sensibilidad se configura desde la tool Config, porque el punto justo
 * depende de dónde esté apoyado: en un escritorio donde se teclea fuerte, la
 * vibración de la mesa puede alcanzar para despertarlo.
 */
#include "ui_power.h"
#include "bsp.h"
#include "qmi8658.h"

#include "lvgl.h"
#include "esp_log.h"
#include "nvs.h"

#include <math.h>
#include <time.h>

static const char *TAG = "ui_power";

#define TICK_MS  100   /* ritmo del IMU con la pantalla apagada */

/* Umbrales por sensibilidad: rotación (grados/s POR ENCIMA del reposo) y
 * cambio de aceleración (g) entre muestras. Alcanza con superar uno.
 *
 * Los valores son bajos porque se comparan contra una línea base, no contra
 * cero: el giroscopio en reposo no marca 0 sino su propio sesgo (unos pocos
 * grados/s, distinto en cada chip). Restando ese piso se puede exigir mucho
 * menos movimiento sin que el ruido despierte la pantalla sola. */
static const struct { float gyro_dps; float accel_g; const char *name; } SENS[] = {
    { 18.0f, 0.10f, "Baja"  },   /* hay que levantarlo                 */
    {  6.0f, 0.035f, "Media" },  /* moverlo sobre la mesa              */
    {  2.5f, 0.015f, "Alta"  },  /* un empujoncito al escritorio       */
};

/* Brillo con el que se enciende durante el modo noche: lo justo para leer la
 * hora sin encandilarse. */
#define NIGHT_BRIGHTNESS 15

#define PWR_NS      "uipower"
#define KEY_BRIGHT  "bright"
#define KEY_MOTION  "motion"
#define KEY_SENS    "sens"
#define KEY_SLEEP   "sleep_s"
#define KEY_NIGHT   "night"
#define KEY_NIGHT_S "night_s"
#define KEY_NIGHT_E "night_e"

static lv_timer_t *s_timer = NULL;
static bool s_asleep = false;
static int  s_brightness = 100;
static bool s_motion_wake = true;
/* Alta por defecto: el reloj vive en el escritorio y sin batería, así que
 * interesa que responda al menor movimiento, no ahorrar energía. */
static int  s_sens = UI_POWER_SENS_ALTA;
static int  s_sleep_s = 45;      /* 0 = no apagar nunca */
static bool s_inhibit = false;   /* algo pide que no se duerma (ver linterna) */
static bool s_night = false;     /* modo noche habilitado */
static bool s_mute = false;      /* silencio manual, no persistido */
static int  s_night_start = 23;
static int  s_night_end = 7;

/* Última lectura del acelerómetro, para medir el cambio entre muestras. */
static float s_pax = 0, s_pay = 0, s_paz = 0;
static bool  s_have_prev = false;

/* Línea base de rotación: el reposo del giroscopio no es 0 sino su sesgo. Se
 * sigue lentamente para poder exigir umbrales chicos sin falsos despertares. */
static float s_gyro_base = 0.0f;
static bool  s_have_base = false;

/* --------------------------------- NVS ---------------------------------- */

static void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open(PWR_NS, NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, KEY_BRIGHT, &v) == ESP_OK && v >= 10 && v <= 100) s_brightness = (int)v;
    if (nvs_get_i32(h, KEY_MOTION, &v) == ESP_OK) s_motion_wake = (v != 0);
    if (nvs_get_i32(h, KEY_SENS, &v) == ESP_OK && v >= 0 && v < UI_POWER_SENS_COUNT) s_sens = (int)v;
    if (nvs_get_i32(h, KEY_SLEEP, &v) == ESP_OK && v >= 0 && v <= 3600) s_sleep_s = (int)v;
    if (nvs_get_i32(h, KEY_NIGHT, &v) == ESP_OK) s_night = (v != 0);
    if (nvs_get_i32(h, KEY_NIGHT_S, &v) == ESP_OK && v >= 0 && v <= 23) s_night_start = (int)v;
    if (nvs_get_i32(h, KEY_NIGHT_E, &v) == ESP_OK && v >= 0 && v <= 23) s_night_end = (int)v;
    nvs_close(h);
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(PWR_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, KEY_BRIGHT, (int32_t)s_brightness);
    nvs_set_i32(h, KEY_MOTION, s_motion_wake ? 1 : 0);
    nvs_set_i32(h, KEY_SENS,   (int32_t)s_sens);
    nvs_set_i32(h, KEY_SLEEP,  (int32_t)s_sleep_s);
    nvs_set_i32(h, KEY_NIGHT,  s_night ? 1 : 0);
    nvs_set_i32(h, KEY_NIGHT_S, (int32_t)s_night_start);
    nvs_set_i32(h, KEY_NIGHT_E, (int32_t)s_night_end);
    nvs_commit(h);
    nvs_close(h);
}

/* ------------------------------- Modo noche ------------------------------ */

bool ui_power_night_now(void)
{
    if (!s_night) return false;

    time_t now = time(NULL);
    if (now < 1600000000) return false;   /* sin hora válida no se silencia nada */

    struct tm tm;
    localtime_r(&now, &tm);
    int h = tm.tm_hour;

    if (s_night_start == s_night_end) return false;        /* franja vacía */
    if (s_night_start < s_night_end) {                     /* p.ej. 1 -> 7 */
        return h >= s_night_start && h < s_night_end;
    }
    return h >= s_night_start || h < s_night_end;          /* cruza medianoche */
}

void ui_power_set_mute(bool on) { s_mute = on; }
bool ui_power_get_mute(void)    { return s_mute; }

bool ui_power_quiet_now(void)   { return s_mute || ui_power_night_now(); }

/* Brillo con el que corresponde encender ahora. */
static int wake_brightness(void)
{
    if (!ui_power_night_now()) return s_brightness;
    return (s_brightness < NIGHT_BRIGHTNESS) ? s_brightness : NIGHT_BRIGHTNESS;
}

/* ------------------------------ Dormir/despertar ------------------------ */

static void screen_sleep(void)
{
    if (s_asleep) return;
    s_asleep = true;
    s_have_prev = false;        /* la referencia de movimiento arranca limpia */
    s_have_base = false;
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
    bsp_backlight_set(wake_brightness());   /* atenuado si es de noche */
    ESP_LOGI(TAG, "Pantalla encendida");
}

bool ui_power_asleep(void) { return s_asleep; }

void ui_power_inhibit(bool on)
{
    s_inhibit = on;
    /* Al activarlo, encender ya; al soltarlo, reiniciar el conteo para no
     * apagar de golpe apenas se cierra la tool. */
    ui_power_wake();
}

/* -------------------------------- Movimiento ---------------------------- */

static bool motion_detected(void)
{
    if (!qmi8658_available()) return false;

    /* El giroscopio es lo más sensible a que alguien toque el equipo: una
     * rotación mínima se ve acá antes que en la gravedad. Se compara contra la
     * línea base (el sesgo del chip en reposo), no contra cero. */
    float gx, gy, gz;
    if (qmi8658_read_gyro(&gx, &gy, &gz) == ESP_OK) {
        float rot = fabsf(gx) + fabsf(gy) + fabsf(gz);

        if (!s_have_base) {                 /* primera muestra tras dormirse */
            s_gyro_base = rot;
            s_have_base = true;
        } else if (rot - s_gyro_base > SENS[s_sens].gyro_dps) {
            return true;
        } else {
            /* Seguimiento lento: solo se adapta cuando está quieto, así una
             * vibración sostenida no termina "normalizándose". */
            s_gyro_base += 0.05f * (rot - s_gyro_base);
        }
    }

    float ax, ay, az;
    if (qmi8658_read_accel(&ax, &ay, &az) != ESP_OK) return false;

    if (!s_have_prev) {         /* primera muestra: solo tomar referencia */
        s_pax = ax; s_pay = ay; s_paz = az;
        s_have_prev = true;
        return false;
    }

    float d = fabsf(ax - s_pax) + fabsf(ay - s_pay) + fabsf(az - s_paz);
    s_pax = ax; s_pay = ay; s_paz = az;
    return d > SENS[s_sens].accel_g;
}

/* --------------------------------- Tick --------------------------------- */

static void tick_cb(lv_timer_t *t)
{
    (void)t;

    uint32_t idle = lv_disp_get_inactive_time(NULL);
    uint32_t sleep_ms = (uint32_t)s_sleep_s * 1000u;

    if (!s_asleep) {
        if (!s_inhibit && s_sleep_s > 0 && idle > sleep_ms) screen_sleep();
        return;
    }

    /* Dormida: tocar la pantalla reinicia el contador de inactividad de LVGL,
     * así que un toque siempre la despierta, haya o no IMU. */
    if (idle < sleep_ms) {
        ui_power_wake();
        return;
    }

    if (s_motion_wake && motion_detected()) {
        ESP_LOGI(TAG, "Movimiento detectado");
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
    ESP_LOGI(TAG, "Energía lista (apaga a los %d s, movimiento %s/%s)",
             s_sleep_s, s_motion_wake ? "on" : "off", SENS[s_sens].name);
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

void ui_power_set_motion_wake(bool on)
{
    s_motion_wake = on;
    s_have_prev = false;
    s_have_base = false;
    settings_save();
}

bool ui_power_get_motion_wake(void) { return s_motion_wake; }

void ui_power_set_sensitivity(ui_power_sens_t s)
{
    if (s < 0 || s >= UI_POWER_SENS_COUNT) return;
    s_sens = (int)s;
    s_have_prev = false;
    s_have_base = false;
    settings_save();
}

ui_power_sens_t ui_power_get_sensitivity(void) { return (ui_power_sens_t)s_sens; }

const char *ui_power_sens_name(ui_power_sens_t s)
{
    if (s < 0 || s >= UI_POWER_SENS_COUNT) return "?";
    return SENS[s].name;
}

void ui_power_set_night(bool on)
{
    s_night = on;
    if (!s_asleep) bsp_backlight_set(wake_brightness());
    settings_save();
}

bool ui_power_get_night(void) { return s_night; }

void ui_power_set_night_range(int start_h, int end_h)
{
    if (start_h < 0 || start_h > 23 || end_h < 0 || end_h > 23) return;
    s_night_start = start_h;
    s_night_end = end_h;
    if (!s_asleep) bsp_backlight_set(wake_brightness());
    settings_save();
}

int ui_power_get_night_start(void) { return s_night_start; }
int ui_power_get_night_end(void)   { return s_night_end; }

void ui_power_set_sleep_s(int seconds)
{
    if (seconds < 0)    seconds = 0;
    if (seconds > 3600) seconds = 3600;
    s_sleep_s = seconds;
    if (s_sleep_s == 0) ui_power_wake();   /* "nunca": encenderla ya */
    settings_save();
}

int ui_power_get_sleep_s(void) { return s_sleep_s; }
