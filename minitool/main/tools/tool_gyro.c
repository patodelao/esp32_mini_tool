/*
 * tool_gyro.c — Orientación con el IMU: pitch/roll desde el acelerómetro y
 * rumbo relativo (yaw) integrando el giroscopio Z. Un punto orbita el borde
 * indicando el giro acumulado; tocar "Cero" lo reinicia.
 *
 * El punto deja detrás el ARCO que barrió: un solo arco cian, con las puntas
 * redondeadas, que crece desde el cero hasta donde estás. De un vistazo se ve
 * cuánto giraste sin leer el número.
 *
 * Un intento anterior dibujaba la estela con 24 segmentos de arco superpuestos
 * para lograr un degradé. Se veía mal y, peor, iba a los tirones: son 24
 * objetos del tamaño de la pantalla invalidándose 25 veces por segundo, y
 * LVGL no da abasto. Un arco único cuesta prácticamente nada y se mueve fluido,
 * que era el punto.
 *
 * La velocidad angular se filtra antes de integrarla (media móvil
 * exponencial). El giroscopio tiene ruido y sin filtrar el punto tiembla
 * aunque el reloj esté quieto.
 *
 * Nota: sin magnetómetro el yaw es relativo (deriva lentamente); sirve para
 * medir giros, no como brújula absoluta.
 */
#include "tool.h"
#include "qmi8658.h"

#include <math.h>
#include <stdio.h>

/* Misma calibración de montaje que tool_level */
#define IMU_SIGN_X (-1.0f)
#define IMU_SIGN_Y (-1.0f)
#define IMU_SIGN_Z (-1.0f)

#define RING_R      92
#define TICK_MS     40

/* Peso de la lectura nueva en el filtro. Más bajo = más suave y más lento en
 * reaccionar; 0.30 saca el temblor sin que se sienta pesado. */
#define GZ_SMOOTH   0.30f

static lv_obj_t *s_dot = NULL;
static lv_obj_t *s_sweep = NULL;       /* el arco barrido */
static lv_obj_t *s_pr_label = NULL;
static lv_obj_t *s_yaw_label = NULL;
static lv_timer_t *s_timer = NULL;
static float s_yaw = 0;
static float s_gz = 0;                 /* velocidad angular ya filtrada */

static void update_cb(lv_timer_t *t)
{
    (void)t;
    float ax, ay, az, gx, gy, gz;
    if (qmi8658_read_accel(&ax, &ay, &az) != ESP_OK) return;
    if (qmi8658_read_gyro(&gx, &gy, &gz) != ESP_OK) return;
    ax *= IMU_SIGN_X; ay *= IMU_SIGN_Y; az *= IMU_SIGN_Z;
    gz *= IMU_SIGN_Z;

    /* Pitch/roll desde la gravedad */
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    float roll = atan2f(ay, az) * 180.0f / (float)M_PI;

    /* Yaw relativo integrando el gyro Z, con la velocidad filtrada primero */
    s_gz += (gz - s_gz) * GZ_SMOOTH;
    s_yaw += s_gz * (TICK_MS / 1000.0f);
    while (s_yaw >= 360.0f) s_yaw -= 360.0f;
    while (s_yaw < 0.0f) s_yaw += 360.0f;

    /* Punto orbitando el borde según yaw, con el arco barrido detrás */
    float rad = s_yaw * (float)M_PI / 180.0f;
    lv_obj_align(s_dot, LV_ALIGN_CENTER,
                 (lv_coord_t)(sinf(rad) * RING_R),
                 (lv_coord_t)(-cosf(rad) * RING_R));
    lv_arc_set_value(s_sweep, (int)(s_yaw * 10.0f));   /* décimas: sin escalones */

    char buf[48];
    snprintf(buf, sizeof(buf), "P %+.1f\xC2\xB0  R %+.1f\xC2\xB0", (double)pitch, (double)roll);
    lv_label_set_text(s_pr_label, buf);
    snprintf(buf, sizeof(buf), "%.0f\xC2\xB0", (double)s_yaw);
    lv_label_set_text(s_yaw_label, buf);
}

static void zero_cb(lv_event_t *e)
{
    (void)e;
    s_yaw = 0;
    s_gz = 0;
    if (s_sweep) lv_arc_set_value(s_sweep, 0);
}

static void gyro_open(lv_obj_t *parent)
{
    if (!qmi8658_available()) {
        lv_obj_t *msg = lv_label_create(parent);
        lv_label_set_text(msg, "IMU no detectado");
        lv_obj_set_style_text_color(msg, lv_color_hex(0xE74C3C), 0);
        lv_obj_center(msg);
        return;
    }

    /* Anillo guía */
    lv_obj_t *ring = lv_obj_create(parent);
    lv_obj_remove_style_all(ring);
    lv_obj_set_size(ring, 2 * RING_R + 12, 2 * RING_R + 12);
    lv_obj_center(ring);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(ring, 2, 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0x1B5A8E), 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);

    /* Arco barrido: UN solo objeto. Su indicador crece desde el cero (arriba)
     * hasta el ángulo actual. El fondo va transparente porque el anillo guía ya
     * está dibujado detrás. */
    s_sweep = lv_arc_create(parent);
    lv_obj_remove_style(s_sweep, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_sweep, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(s_sweep, 2 * RING_R + 12, 2 * RING_R + 12);
    lv_obj_center(s_sweep);
    lv_arc_set_rotation(s_sweep, 270);           /* el 0 arriba, como el punto */
    lv_arc_set_bg_angles(s_sweep, 0, 360);
    lv_arc_set_range(s_sweep, 0, 3600);          /* décimas de grado */
    lv_arc_set_value(s_sweep, 0);
    lv_obj_set_style_arc_opa(s_sweep, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_sweep, 7, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_sweep, true, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_sweep, lv_color_hex(0x4AE8FF), LV_PART_INDICATOR);

    /* Punto indicador de yaw */
    s_dot = lv_obj_create(parent);
    lv_obj_remove_style_all(s_dot);
    lv_obj_set_size(s_dot, 16, 16);
    lv_obj_set_style_radius(s_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_dot, lv_color_hex(0x4AA8FF), 0);
    lv_obj_align(s_dot, LV_ALIGN_CENTER, 0, -RING_R);

    /* Yaw grande al centro */
    s_yaw_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_yaw_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_yaw_label, lv_color_white(), 0);
    lv_label_set_text(s_yaw_label, "0\xC2\xB0");
    lv_obj_align(s_yaw_label, LV_ALIGN_CENTER, 0, -18);

    /* Pitch / roll */
    s_pr_label = lv_label_create(parent);
    lv_obj_set_style_text_color(s_pr_label, lv_color_hex(0xBFE0FF), 0);
    lv_label_set_text(s_pr_label, "");
    lv_obj_align(s_pr_label, LV_ALIGN_CENTER, 0, 16);

    /* Botón de cero */
    lv_obj_t *zero = lv_btn_create(parent);
    lv_obj_set_size(zero, 82, 32);
    lv_obj_align(zero, LV_ALIGN_CENTER, 0, 58);
    lv_obj_set_style_radius(zero, 16, 0);
    lv_obj_set_style_bg_color(zero, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(zero, zero_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(zero);
    lv_label_set_text(lbl, "Cero");
    lv_obj_center(lbl);

    s_yaw = 0;
    s_gz = 0;
    s_timer = lv_timer_create(update_cb, TICK_MS, NULL);
}

static void gyro_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    s_dot = NULL;
    s_pr_label = NULL;
    s_yaw_label = NULL;
    s_sweep = NULL;
}

const tool_t tool_gyro = {
    .name = "Giro",
    .icon = LV_SYMBOL_REFRESH,
    .accent = 0x4AD9D9,
    .open = gyro_open,
    .close = gyro_close,
};
