/*
 * pedometer_service.c — Detección de pasos por acelerómetro (lv_timer global).
 *
 * Algoritmo: se calcula la magnitud |a|, se le resta una línea base lenta para
 * quedarnos con la oscilación de la marcha, y una máquina de estados con
 * histéresis + antirrebote cuenta un paso por cada pico válido.
 */
#include "pedometer_service.h"
#include "qmi8658.h"

#include "lvgl.h"
#include "nvs.h"
#include "esp_log.h"

#include <math.h>

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

static void save_steps(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_STEPS, s_steps);
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
        if (nvs_get_u32(h, NVS_KEY_STEPS, &v) == ESP_OK) s_steps = v;
        nvs_close(h);
    }
    s_saved = s_steps;
}

static void sample_cb(lv_timer_t *t)
{
    (void)t;
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
    save_steps();
}
