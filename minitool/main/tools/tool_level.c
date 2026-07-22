/*
 * tool_level.c — Nivel de burbuja usando el acelerómetro del QMI8658.
 *
 * La burbuja se desplaza hacia el lado alto de la placa (como un nivel real)
 * y el texto central muestra la inclinación total en grados. Verde = nivelado.
 *
 * Calibración: coloca el dispositivo sobre una superficie plana y pulsa
 * "Calibrar". Se captura el sesgo actual del acelerómetro (X/Y) como el nuevo
 * cero y se guarda en NVS, de modo que sobrevive a reinicios. Esto corrige el
 * "descalibrado" cuando la placa/IMU no queda perfectamente perpendicular a la
 * gravedad en reposo.
 *
 * Nota: si la burbuja se mueve al revés en algún eje, basta invertir el signo
 * en IMU_SIGN_X / IMU_SIGN_Y (la orientación de montaje del IMU puede variar).
 */
#include "tool.h"
#include "qmi8658.h"

#include "nvs.h"
#include "esp_log.h"

#include <math.h>
#include <stdio.h>

static const char *TAG = "level";

#define BUBBLE_SIZE   26
#define BUBBLE_MAX_R  80   /* desplazamiento máximo desde el centro (px) */
#define TILT_GAIN     220.0f /* px por unidad de seno de inclinación */
#define LEVEL_THRESH  1.5f /* grados para considerarse "nivelado" */

#define CALIB_SAMPLES 32   /* muestras promediadas al calibrar */

/* Persistencia del cero de calibración en NVS */
#define NVS_NS        "storage"
#define NVS_KEY_AX0   "lvl_ax0"
#define NVS_KEY_AY0   "lvl_ay0"
#define NVS_SCALE     100000.0f /* float <-> i32 con 5 decimales */

/*
 * Calibración de montaje del IMU en esta placa (verificado en hardware):
 * con la pantalla hacia arriba az lee ≈ -1g (chip montado invertido) y los
 * ejes X/Y quedan espejados respecto a la pantalla. Ajustar aquí si cambia.
 */
#define IMU_SIGN_X (-1.0f)
#define IMU_SIGN_Y (-1.0f)
#define IMU_SIGN_Z (-1.0f)

static lv_obj_t *s_bubble = NULL;
static lv_obj_t *s_deg_label = NULL;
static lv_obj_t *s_calib_btn = NULL;
static lv_obj_t *s_calib_lbl = NULL;
static lv_timer_t *s_timer = NULL;
static lv_timer_t *s_toast_timer = NULL;
static float s_fx = 0, s_fy = 0; /* filtro suavizado de la posición */

/* Sesgo de cero (en g, ya con signo de montaje aplicado) */
static float s_ax0 = 0.0f, s_ay0 = 0.0f;

/* --- Persistencia ---------------------------------------------------------- */

static void load_offsets(void)
{
    s_ax0 = s_ay0 = 0.0f;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 0;
        if (nvs_get_i32(h, NVS_KEY_AX0, &v) == ESP_OK) s_ax0 = (float)v / NVS_SCALE;
        if (nvs_get_i32(h, NVS_KEY_AY0, &v) == ESP_OK) s_ay0 = (float)v / NVS_SCALE;
        nvs_close(h);
        ESP_LOGI(TAG, "Offsets cargados: ax0=%.4f ay0=%.4f", (double)s_ax0, (double)s_ay0);
    }
}

static void save_offsets(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, NVS_KEY_AX0, (int32_t)lroundf(s_ax0 * NVS_SCALE));
        nvs_set_i32(h, NVS_KEY_AY0, (int32_t)lroundf(s_ay0 * NVS_SCALE));
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Offsets guardados: ax0=%.4f ay0=%.4f", (double)s_ax0, (double)s_ay0);
    }
}

/* --- Lógica --------------------------------------------------------------- */

static void update_cb(lv_timer_t *t)
{
    (void)t;
    float ax, ay, az;
    if (qmi8658_read_accel(&ax, &ay, &az) != ESP_OK) return;
    ax *= IMU_SIGN_X;
    ay *= IMU_SIGN_Y;
    az *= IMU_SIGN_Z;

    /* Restar el cero de calibración a la componente horizontal */
    ax -= s_ax0;
    ay -= s_ay0;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.05f) return; /* lectura inválida / caída libre */

    /* Inclinación desde la horizontal calibrada: asin de la componente
       horizontal (más directo y consistente con la burbuja que acos(az)). */
    float h = sqrtf(ax * ax + ay * ay) / norm;
    if (h > 1.0f) h = 1.0f;
    float tilt_deg = asinf(h) * 180.0f / (float)M_PI;

    /* La burbuja va hacia el lado alto: opuesta a la proyección de la gravedad */
    float px = -(ax / norm) * TILT_GAIN;
    float py =  (ay / norm) * TILT_GAIN;

    /* Limitar al círculo guía */
    float r = sqrtf(px * px + py * py);
    if (r > BUBBLE_MAX_R) {
        px *= BUBBLE_MAX_R / r;
        py *= BUBBLE_MAX_R / r;
    }

    /* Suavizado exponencial para que no tiemble */
    s_fx += 0.35f * (px - s_fx);
    s_fy += 0.35f * (py - s_fy);

    bool level = tilt_deg < LEVEL_THRESH;
    lv_obj_align(s_bubble, LV_ALIGN_CENTER, (lv_coord_t)s_fx, (lv_coord_t)s_fy);
    lv_obj_set_style_bg_color(s_bubble, level ? lv_color_hex(0x35D07F)
                                              : lv_color_hex(0xFFC03A), 0);

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f" "\xC2\xB0", (double)tilt_deg); /* ° en UTF-8 */
    lv_label_set_text(s_deg_label, buf);
    lv_obj_set_style_text_color(s_deg_label, level ? lv_color_hex(0x35D07F)
                                                   : lv_color_white(), 0);
}

static void toast_hide_cb(lv_timer_t *t)
{
    (void)t;
    if (s_calib_lbl) lv_obj_add_flag(s_calib_lbl, LV_OBJ_FLAG_HIDDEN);
    s_toast_timer = NULL;
}

/* Promedia varias muestras y las fija como nuevo cero horizontal. */
static void calibrate_now(lv_event_t *e)
{
    (void)e;
    float sx = 0, sy = 0;
    int n = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        float ax, ay, az;
        if (qmi8658_read_accel(&ax, &ay, &az) == ESP_OK) {
            sx += ax * IMU_SIGN_X;
            sy += ay * IMU_SIGN_Y;
            n++;
        }
    }
    if (n == 0) return;

    s_ax0 = sx / n;
    s_ay0 = sy / n;
    save_offsets();

    /* Reset del filtro para que la burbuja salte al centro de inmediato */
    s_fx = s_fy = 0;

    if (s_calib_lbl) {
        lv_label_set_text(s_calib_lbl, LV_SYMBOL_OK " Calibrado");
        lv_obj_clear_flag(s_calib_lbl, LV_OBJ_FLAG_HIDDEN);
        if (s_toast_timer) lv_timer_reset(s_toast_timer);
        else s_toast_timer = lv_timer_create(toast_hide_cb, 1200, NULL);
        lv_timer_set_repeat_count(s_toast_timer, 1);
    }
}

/* Círculo guía concéntrico, solo borde. */
static void create_ring(lv_obj_t *parent, int diameter, lv_color_t color)
{
    lv_obj_t *ring = lv_obj_create(parent);
    lv_obj_remove_style_all(ring);
    lv_obj_set_size(ring, diameter, diameter);
    lv_obj_center(ring);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(ring, 2, 0);
    lv_obj_set_style_border_color(ring, color, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);
}

static void level_open(lv_obj_t *parent)
{
    if (!qmi8658_available()) {
        lv_obj_t *msg = lv_label_create(parent);
        lv_label_set_text(msg, "IMU no detectado");
        lv_obj_set_style_text_color(msg, lv_color_hex(0xFF5A5A), 0);
        lv_obj_center(msg);
        return;
    }

    load_offsets();

    /* Anillos guía */
    create_ring(parent, 2 * BUBBLE_MAX_R + BUBBLE_SIZE, lv_color_hex(0x2E82C8));
    create_ring(parent, BUBBLE_MAX_R + BUBBLE_SIZE, lv_color_hex(0x1B5A8E));
    create_ring(parent, 44, lv_color_hex(0x35D07F));

    /* Burbuja */
    s_bubble = lv_obj_create(parent);
    lv_obj_remove_style_all(s_bubble);
    lv_obj_set_size(s_bubble, BUBBLE_SIZE, BUBBLE_SIZE);
    lv_obj_set_style_radius(s_bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_bubble, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_bubble, lv_color_hex(0x35D07F), 0);
    lv_obj_center(s_bubble);

    /* Grados en el centro */
    s_deg_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_deg_label, &lv_font_montserrat_16, 0);
    lv_label_set_text(s_deg_label, "--");
    lv_obj_align(s_deg_label, LV_ALIGN_CENTER, 0, 58);

    /* Botón de calibración (abajo, sin tapar la burbuja) */
    s_calib_btn = lv_btn_create(parent);
    lv_obj_set_size(s_calib_btn, 108, 32);
    lv_obj_align(s_calib_btn, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_radius(s_calib_btn, 16, 0);
    lv_obj_set_style_bg_color(s_calib_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_calib_btn, calibrate_now, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_lbl = lv_label_create(s_calib_btn);
    lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(btn_lbl, LV_SYMBOL_GPS " Calibrar");
    lv_obj_center(btn_lbl);

    /* Toast de confirmación (oculto hasta calibrar) */
    s_calib_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_calib_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_calib_lbl, lv_color_hex(0x35D07F), 0);
    lv_label_set_text(s_calib_lbl, LV_SYMBOL_OK " Calibrado");
    lv_obj_align(s_calib_lbl, LV_ALIGN_CENTER, 0, 82);
    lv_obj_add_flag(s_calib_lbl, LV_OBJ_FLAG_HIDDEN);

    s_fx = s_fy = 0;
    s_timer = lv_timer_create(update_cb, 40, NULL); /* ~25 fps */
}

static void level_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    if (s_toast_timer) {
        lv_timer_del(s_toast_timer);
        s_toast_timer = NULL;
    }
    s_bubble = NULL;
    s_deg_label = NULL;
    s_calib_btn = NULL;
    s_calib_lbl = NULL;
}

const tool_t tool_level = {
    .name = "Nivel",
    .icon = LV_SYMBOL_GPS,
    .accent = 0x35D07F,
    .open = level_open,
    .close = level_close,
};
