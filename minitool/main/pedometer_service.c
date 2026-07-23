/*
 * pedometer_service.c — Detección de pasos por acelerómetro (lv_timer global).
 *
 * Algoritmo: se calcula la magnitud |a|, se le resta una línea base lenta para
 * quedarnos con la oscilación de la marcha, y una máquina de estados con
 * histéresis + antirrebote cuenta un paso por cada pico válido.
 */
#include "pedometer_service.h"
#include "qmi8658.h"
#include "ui_notify.h"

#include "lvgl.h"
#include "nvs.h"
#include "esp_log.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

static const char *TAG = "pedometer";

#define SAMPLE_MS       30      /* ~33 Hz de muestreo                       */
#define TH_HI           0.11f   /* umbral de subida (g) para armar el pico  */
#define TH_LO           0.04f   /* umbral de bajada (g) para rearmar        */
#define MIN_STEP_MS     260     /* antirrebote: paso mínimo cada 260 ms     */
#define SAVE_EVERY      40      /* persistir cada N pasos nuevos            */

#define NVS_NS          "storage"
#define NVS_KEY_STEPS   "ped_steps"

static lv_timer_t *s_timer = NULL;
static uint32_t    s_steps = 0;
static uint32_t    s_saved = 0;

/* Estado del detector */
static float    s_base = 1.0f;   /* línea base (~1g) */
static bool     s_armed = false; /* dentro de un pico */
static uint32_t s_last_step_ms = 0;

/* --- Día y historial -------------------------------------------------------
 *
 * El contador es de HOY: al cambiar el día se cierra y pasa al historial. Un
 * total que crece para siempre no dice nada sobre si te moviste esta semana.
 *
 * El día se identifica igual que en sensor_service (año*366 + día del año), y
 * si el reloj todavía no está en hora no se cierra nada: mejor sumar de más
 * que inventar un día vacío. */
#define NVS_KEY_DAY     "ped_day"
#define NVS_KEY_HIST    "ped_hist"
#define NVS_KEY_GOAL    "ped_goal"

static uint32_t s_hist[PEDOMETER_DAYS];   /* días cerrados, el 0 es el más viejo */
static int      s_hist_n = 0;
static int32_t  s_day = -1;
static uint32_t s_goal = 8000;
static bool     s_goal_notified = false;

static int32_t current_day(void)
{
    time_t now = time(NULL);
    if (now < 1600000000) return -1;
    struct tm tm;
    localtime_r(&now, &tm);
    return (int32_t)(tm.tm_year * 366 + tm.tm_yday);
}

static void save_steps(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_STEPS, s_steps);
        nvs_set_i32(h, NVS_KEY_DAY, s_day);
        nvs_set_u32(h, NVS_KEY_GOAL, s_goal);
        nvs_set_blob(h, NVS_KEY_HIST, s_hist, (size_t)s_hist_n * sizeof(s_hist[0]));
        nvs_commit(h);
        nvs_close(h);
        s_saved = s_steps;
    }
}

static void load_steps(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint32_t v = 0;
        int32_t d = -1;
        if (nvs_get_u32(h, NVS_KEY_STEPS, &v) == ESP_OK) s_steps = v;
        if (nvs_get_i32(h, NVS_KEY_DAY, &d) == ESP_OK) s_day = d;
        if (nvs_get_u32(h, NVS_KEY_GOAL, &v) == ESP_OK) s_goal = v;
        size_t sz = sizeof(s_hist);
        if (nvs_get_blob(h, NVS_KEY_HIST, s_hist, &sz) == ESP_OK) {
            s_hist_n = (int)(sz / sizeof(s_hist[0]));
        }
        nvs_close(h);
    }
    s_saved = s_steps;
}

/* Cierra el día si cambió: lo empuja al historial y arranca de cero. */
static void roll_day_if_needed(void)
{
    int32_t day = current_day();
    if (day < 0) return;                 /* sin hora válida, no se cierra nada */

    if (s_day < 0) { s_day = day; return; }   /* primera vez con hora */
    if (day == s_day) return;

    if (s_hist_n < PEDOMETER_DAYS) {
        s_hist[s_hist_n++] = s_steps;
    } else {
        for (int i = 1; i < PEDOMETER_DAYS; i++) s_hist[i - 1] = s_hist[i];
        s_hist[PEDOMETER_DAYS - 1] = s_steps;
    }
    ESP_LOGI(TAG, "Día cerrado con %lu pasos", (unsigned long)s_steps);

    s_steps = 0;
    s_day = day;
    s_goal_notified = false;
    save_steps();
}

static void sample_cb(lv_timer_t *t)
{
    (void)t;

    /* Barato: solo mira el reloj, y cierra el día como mucho una vez. */
    roll_day_if_needed();

    if (!qmi8658_available()) return;

    float ax, ay, az;
    if (qmi8658_read_accel(&ax, &ay, &az) != ESP_OK) return;

    float mag = sqrtf(ax * ax + ay * ay + az * az);

    /* Línea base lenta: sigue la gravedad y deriva del sensor */
    s_base += 0.02f * (mag - s_base);
    float ac = mag - s_base; /* oscilación en torno a 0 */

    uint32_t now = lv_tick_get();

    if (!s_armed && ac > TH_HI) {
        s_armed = true;
        if (now - s_last_step_ms >= MIN_STEP_MS) {
            s_steps++;
            s_last_step_ms = now;
            if (s_steps - s_saved >= SAVE_EVERY) save_steps();

            /* Meta cumplida: un aviso, una vez por día. */
            if (s_goal > 0 && !s_goal_notified && s_steps >= s_goal) {
                s_goal_notified = true;
                char msg[48];
                snprintf(msg, sizeof(msg), "%lu pasos hoy", (unsigned long)s_steps);
                ui_notify_push("Meta", NOTIFY_SUCCESS, msg);
            }
        }
    } else if (s_armed && ac < TH_LO) {
        s_armed = false;
    }
}

void pedometer_service_init(void)
{
    if (s_timer) return;
    load_steps();
    s_timer = lv_timer_create(sample_cb, SAMPLE_MS, NULL);
    ESP_LOGI(TAG, "Podómetro iniciado (%lu pasos guardados)", (unsigned long)s_steps);
}

uint32_t pedometer_steps(void) { return s_steps; }

void pedometer_reset(void)
{
    s_steps = 0;
    s_armed = false;
    s_goal_notified = false;
    save_steps();
}

int pedometer_history(uint32_t *out, int max)
{
    if (!out) return 0;
    int n = s_hist_n < max ? s_hist_n : max;
    for (int i = 0; i < n; i++) out[i] = s_hist[s_hist_n - n + i];
    return n;
}

void pedometer_set_goal(uint32_t steps)
{
    s_goal = steps;
    s_goal_notified = (s_goal > 0 && s_steps >= s_goal);
    save_steps();
}

uint32_t pedometer_get_goal(void) { return s_goal; }
