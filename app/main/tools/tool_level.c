/*
 * tool_level.c — Nivel de burbuja usando el acelerómetro del QMI8658.
 *
 * La burbuja se desplaza hacia el lado alto de la placa (como un nivel real)
 * y el texto central muestra la inclinación total en grados. Verde = nivelado.
 *
 * Nota: si la burbuja se mueve al revés en algún eje, basta invertir el signo
 * en el cálculo de px/py (la orientación de montaje del IMU puede variar).
 */
#include "tool.h"
#include "qmi8658.h"

#include <math.h>
#include <stdio.h>

#define BUBBLE_SIZE   26
#define BUBBLE_MAX_R  80   /* desplazamiento máximo desde el centro (px) */
#define TILT_GAIN     220.0f /* px por unidad de seno de inclinación */
#define LEVEL_THRESH  1.5f /* grados para considerarse "nivelado" */

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
static lv_timer_t *s_timer = NULL;
static float s_fx = 0, s_fy = 0; /* filtro suavizado de la posición */

static void update_cb(lv_timer_t *t)
{
    (void)t;
    float ax, ay, az;
    if (qmi8658_read_accel(&ax, &ay, &az) != ESP_OK) return;
    ax *= IMU_SIGN_X;
    ay *= IMU_SIGN_Y;
    az *= IMU_SIGN_Z;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.05f) return; /* lectura inválida / caída libre */

    /* Inclinación total respecto a la horizontal (0° = pantalla hacia arriba) */
    float cz = az / norm;
    if (cz > 1.0f) cz = 1.0f;
    if (cz < -1.0f) cz = -1.0f;
    float tilt_deg = acosf(cz) * 180.0f / (float)M_PI;

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
    lv_obj_align(s_deg_label, LV_ALIGN_CENTER, 0, 60);

    s_fx = s_fy = 0;
    s_timer = lv_timer_create(update_cb, 40, NULL); /* ~25 fps */
}

static void level_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    s_bubble = NULL;
    s_deg_label = NULL;
}

const tool_t tool_level = {
    .name = "Nivel",
    .icon = LV_SYMBOL_GPS,
    .accent = 0x35D07F,
    .open = level_open,
    .close = level_close,
};
