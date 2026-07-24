/*
 * tool_gyro.c — Orientación con el IMU: pitch/roll desde el acelerómetro y
 * rumbo relativo (yaw) integrando el giroscopio Z. Un punto orbita el borde
 * indicando el giro acumulado; tocar "Cero" lo reinicia.
 *
 * El punto va dejando una ESTELA por el arco que barrió. Además de quedar
 * lindo, muestra algo que el punto solo no podía: de un vistazo se ve cuánto
 * giraste y para qué lado, sin leer el número. La estela se degrada del cian
 * al violeta, y las porciones viejas se van apagando.
 *
 * Está hecha con N segmentos de arco superpuestos en vez de un arco único
 * porque LVGL no sabe pintar un arco con degradé; cada segmento lleva su
 * propio color y opacidad, y juntos dan el efecto.
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

/* Segmentos de la estela. Cada uno cubre 360/TRAIL_N grados; con 24 el paso es
 * de 15°, fino como para que se lea continuo y barato de redibujar a 25 fps. */
#define TRAIL_N     24
#define TRAIL_STEP  (360.0f / TRAIL_N)

/* Cuánta estela queda detrás del punto, en segmentos. Media vuelta: más que
 * eso y en un giro completo la cabeza alcanza a la cola y no se distingue
 * dónde empezó. */
#define TRAIL_LEN   12

static lv_obj_t *s_dot = NULL;
static lv_obj_t *s_trail[TRAIL_N];
static lv_obj_t *s_pr_label = NULL;
static lv_obj_t *s_yaw_label = NULL;
static lv_timer_t *s_timer = NULL;
static float s_yaw = 0;

/* Del cian al violeta a lo largo de la estela: la cabeza es la más brillante
 * y la cola se va hacia el fondo. */
static uint32_t trail_color(int age)
{
    float t = (float)age / (float)TRAIL_LEN;      /* 0 = cabeza, 1 = cola */
    int r = (int)(0x4A + (0xB0 - 0x4A) * t);
    int g = (int)(0xE8 - (0xE8 - 0x40) * t);
    int b = (int)(0xFF - (0xFF - 0xE0) * t);
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

/* Pinta los TRAIL_LEN segmentos que quedan detrás de la cabeza y apaga el
 * resto. Se recorre hacia atrás desde el segmento donde está el punto. */
static void trail_update(float yaw)
{
    int head = (int)(yaw / TRAIL_STEP) % TRAIL_N;

    for (int i = 0; i < TRAIL_N; i++) {
        if (!s_trail[i]) continue;
        lv_obj_set_style_arc_opa(s_trail[i], LV_OPA_TRANSP, LV_PART_MAIN);
    }

    for (int age = 0; age < TRAIL_LEN; age++) {
        int idx = ((head - age) % TRAIL_N + TRAIL_N) % TRAIL_N;
        if (!s_trail[idx]) continue;

        lv_opa_t opa = (lv_opa_t)(LV_OPA_COVER - (age * (LV_OPA_COVER - 30)) / TRAIL_LEN);
        lv_obj_set_style_arc_color(s_trail[idx], lv_color_hex(trail_color(age)), LV_PART_MAIN);
        lv_obj_set_style_arc_opa(s_trail[idx], opa, LV_PART_MAIN);
    }
}

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

    /* Yaw relativo integrando el gyro Z */
    s_yaw += gz * (TICK_MS / 1000.0f);
    while (s_yaw >= 360.0f) s_yaw -= 360.0f;
    while (s_yaw < 0.0f) s_yaw += 360.0f;

    /* Punto orbitando el borde según yaw, con su estela detrás */
    float rad = s_yaw * (float)M_PI / 180.0f;
    lv_obj_align(s_dot, LV_ALIGN_CENTER,
                 (lv_coord_t)(sinf(rad) * RING_R),
                 (lv_coord_t)(-cosf(rad) * RING_R));
    trail_update(s_yaw);

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

    /* Estela: TRAIL_N arcos fijos, cada uno cubriendo su porción del círculo.
     * Se crean una sola vez y después solo se les cambia color y opacidad, que
     * es mucho más barato que recalcular ángulos a 25 fps. El 0° va arriba,
     * igual que el punto, por eso el -90 de arranque. */
    for (int i = 0; i < TRAIL_N; i++) {
        lv_obj_t *seg = lv_arc_create(parent);
        lv_obj_remove_style(seg, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(seg, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(seg, 2 * RING_R + 12, 2 * RING_R + 12);
        lv_obj_center(seg);
        lv_arc_set_bg_angles(seg, (uint16_t)(i * TRAIL_STEP) - 90,
                                  (uint16_t)((i + 1) * TRAIL_STEP) - 90);
        lv_obj_set_style_arc_width(seg, 6, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(seg, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(seg, LV_OPA_TRANSP, LV_PART_INDICATOR);
        s_trail[i] = seg;
    }

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
    for (int i = 0; i < TRAIL_N; i++) s_trail[i] = NULL;
}

const tool_t tool_gyro = {
    .name = "Giro",
    .icon = LV_SYMBOL_REFRESH,
    .accent = 0x4AD9D9,
    .open = gyro_open,
    .close = gyro_close,
};
