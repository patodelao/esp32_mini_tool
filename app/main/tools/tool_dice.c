/*
 * tool_dice.c — Dado: agita la placa (detección de sacudida con el IMU) o
 * toca la pantalla para lanzar. Animación de números aleatorios que se
 * frena hasta caer en el resultado.
 */
#include "tool.h"
#include "qmi8658.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_random.h"
#include "esp_timer.h"

#define SHAKE_THRESH   0.55f /* |a|-1g que dispara el lanzamiento */
#define SHAKE_COOLDOWN 900   /* ms entre lanzamientos por sacudida */
#define ROLL_TICKS     14    /* pasos de animación */

static lv_obj_t *s_num_label = NULL;
static lv_obj_t *s_hint_label = NULL;
static lv_timer_t *s_timer = NULL;
static int s_roll_left = 0;      /* pasos de animación pendientes */
static int64_t s_last_roll_ms = 0;

static int rand_face(void) { return 1 + (esp_random() % 6); }

static void show_face(int n, bool final)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", n);
    lv_label_set_text(s_num_label, buf);
    lv_obj_set_style_text_color(s_num_label,
                                final ? lv_color_hex(0x35D07F) : lv_color_white(), 0);
}

static void start_roll(void)
{
    int64_t now = esp_timer_get_time() / 1000;
    if (now - s_last_roll_ms < SHAKE_COOLDOWN) return;
    s_last_roll_ms = now;
    s_roll_left = ROLL_TICKS;
    lv_obj_add_flag(s_hint_label, LV_OBJ_FLAG_HIDDEN);
}

static void tick_cb(lv_timer_t *t)
{
    (void)t;

    /* Animación de lanzamiento */
    if (s_roll_left > 0) {
        s_roll_left--;
        show_face(rand_face(), s_roll_left == 0);
        return;
    }

    /* Detección de sacudida */
    if (qmi8658_available()) {
        float ax, ay, az;
        if (qmi8658_read_accel(&ax, &ay, &az) == ESP_OK) {
            float mag = sqrtf(ax * ax + ay * ay + az * az);
            if (fabsf(mag - 1.0f) > SHAKE_THRESH) start_roll();
        }
    }
}

static void tap_cb(lv_event_t *e)
{
    (void)e;
    start_roll();
}

static void dice_open(lv_obj_t *parent)
{
    /* Marco del dado */
    lv_obj_t *frame = lv_obj_create(parent);
    lv_obj_remove_style_all(frame);
    lv_obj_set_size(frame, 110, 110);
    lv_obj_center(frame);
    lv_obj_set_style_radius(frame, 24, 0);
    lv_obj_set_style_bg_color(frame, lv_color_hex(0x0E3A5F), 0);
    lv_obj_set_style_bg_opa(frame, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(frame, 2, 0);
    lv_obj_set_style_border_color(frame, lv_color_hex(0x2E82C8), 0);
    lv_obj_clear_flag(frame, LV_OBJ_FLAG_CLICKABLE);

    s_num_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_num_label, &lv_font_montserrat_28, 0);
    lv_obj_center(s_num_label);
    show_face(rand_face(), true);

    s_hint_label = lv_label_create(parent);
    lv_label_set_text(s_hint_label, "agita o toca");
    lv_obj_set_style_text_color(s_hint_label, lv_color_hex(0x8899AA), 0);
    lv_obj_align(s_hint_label, LV_ALIGN_CENTER, 0, 80);

    /* Toda la pantalla lanza al tocarla */
    lv_obj_add_flag(parent, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(parent, tap_cb, LV_EVENT_CLICKED, NULL);

    s_roll_left = 0;
    s_timer = lv_timer_create(tick_cb, 60, NULL);
}

static void dice_close(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }
    s_num_label = NULL;
    s_hint_label = NULL;
}

const tool_t tool_dice = {
    .name = "Dado",
    .icon = LV_SYMBOL_SHUFFLE,
    .open = dice_open,
    .close = dice_close,
};
