/*
 * tool_dice.c — Dado con caras de puntitos (pips) y modo 1 o 2 dados.
 * Agita la placa (detección de sacudida con el IMU) o toca la pantalla para
 * lanzar. Animación de caras aleatorias que se frena hasta el resultado;
 * con 2 dados se muestra la suma.
 */
#include "tool.h"
#include "qmi8658.h"

#include <math.h>
#include <stdio.h>

#include "esp_random.h"
#include "esp_timer.h"

#define SHAKE_THRESH   0.55f /* |a|-1g que dispara el lanzamiento */
#define SHAKE_COOLDOWN 900   /* ms entre lanzamientos por sacudida */
#define ROLL_TICKS     14    /* pasos de animación */
#define MAX_DICE       2

/*
 * Caras del dado como máscara de bits sobre la grilla 3x3:
 *   0 1 2
 *   3 4 5      bit i = puntito visible en la celda i
 *   6 7 8
 */
static const uint16_t k_face_pips[7] = {
    0x000,                 /* (sin usar) */
    0x010,                 /* 1: centro */
    0x101,                 /* 2: diagonal */
    0x111,                 /* 3: diagonal + centro */
    0x145,                 /* 4: esquinas */
    0x155,                 /* 5: esquinas + centro */
    0x16D,                 /* 6: dos columnas de 3 */
};

static lv_obj_t *s_parent = NULL;   /* pantalla de la tool */
static lv_obj_t *s_area = NULL;     /* contenedor de los dados */
static lv_obj_t *s_pips[MAX_DICE][9] = {0};
static lv_obj_t *s_info_label = NULL; /* pista / suma */
static lv_obj_t *s_mode_btns[2] = {0};
static lv_timer_t *s_timer = NULL;

static int s_num_dice = 1;          /* configurable: 1 o 2 (persiste en sesión) */
static int s_faces[MAX_DICE] = {1, 1};
static int s_roll_left = 0;
static bool s_rolled = false;       /* ya hubo un primer lanzamiento */
static int64_t s_last_roll_ms = 0;

static int rand_face(void) { return 1 + (esp_random() % 6); }

static void set_face(int d, int face, bool final)
{
    s_faces[d] = face;
    uint16_t mask = k_face_pips[face];
    lv_color_t color = final ? lv_color_hex(0x35D07F) : lv_color_white();
    for (int i = 0; i < 9; i++) {
        if (!s_pips[d][i]) continue;
        if (mask & (1 << i)) {
            lv_obj_clear_flag(s_pips[d][i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(s_pips[d][i], color, 0);
        } else {
            lv_obj_add_flag(s_pips[d][i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void show_result(bool final)
{
    if (!s_info_label) return;
    if (!s_rolled) {
        lv_label_set_text(s_info_label, "agita o toca"); /* pista inicial */
        return;
    }
    if (final && s_num_dice == 2) {
        lv_label_set_text_fmt(s_info_label, "Total: %d", s_faces[0] + s_faces[1]);
    } else {
        lv_label_set_text(s_info_label, "");
    }
}

/* Crea un dado (marco redondeado + 9 pips ocultos) dentro de s_area. */
static void create_die(int d, lv_coord_t x_ofs, lv_coord_t size)
{
    lv_obj_t *frame = lv_obj_create(s_area);
    lv_obj_remove_style_all(frame);
    lv_obj_set_size(frame, size, size);
    lv_obj_align(frame, LV_ALIGN_CENTER, x_ofs, 0);
    lv_obj_set_style_radius(frame, size / 5, 0);
    lv_obj_set_style_bg_color(frame, lv_color_hex(0x0E3A5F), 0);
    lv_obj_set_style_bg_opa(frame, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(frame, 2, 0);
    lv_obj_set_style_border_color(frame, lv_color_hex(0xB06AFF), 0);
    lv_obj_clear_flag(frame, LV_OBJ_FLAG_CLICKABLE);

    lv_coord_t step = size * 3 / 11;  /* separación de la grilla */
    lv_coord_t pip = size * 3 / 20;   /* diámetro del puntito */
    for (int i = 0; i < 9; i++) {
        int col = (i % 3) - 1;
        int row = (i / 3) - 1;
        lv_obj_t *p = lv_obj_create(frame);
        lv_obj_remove_style_all(p);
        lv_obj_set_size(p, pip, pip);
        lv_obj_align(p, LV_ALIGN_CENTER, col * step, row * step);
        lv_obj_set_style_radius(p, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
        lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p, LV_OBJ_FLAG_CLICKABLE);
        s_pips[d][i] = p;
    }
}

static void update_mode_styles(void)
{
    for (int i = 0; i < 2; i++) {
        if (!s_mode_btns[i]) continue;
        bool active = (s_num_dice == i + 1);
        lv_obj_set_style_bg_color(s_mode_btns[i], active ? lv_color_hex(0xB06AFF)
                                                         : lv_color_hex(0x22303F), 0);
    }
}

/* (Re)construye el área de dados según s_num_dice. */
static void build_dice_area(void)
{
    for (int d = 0; d < MAX_DICE; d++)
        for (int i = 0; i < 9; i++) s_pips[d][i] = NULL;
    if (s_area) lv_obj_del(s_area);

    s_area = lv_obj_create(s_parent);
    lv_obj_remove_style_all(s_area);
    lv_obj_set_size(s_area, 240, 140);
    lv_obj_align(s_area, LV_ALIGN_CENTER, 0, -14);
    lv_obj_clear_flag(s_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_area, LV_OBJ_FLAG_CLICKABLE); /* el toque cae al padre */

    if (s_num_dice == 1) {
        create_die(0, 0, 110);
    } else {
        create_die(0, -52, 90);
        create_die(1, 52, 90);
    }
    for (int d = 0; d < s_num_dice; d++) set_face(d, s_faces[d], true);
    show_result(true);
    update_mode_styles();
}

static void start_roll(void)
{
    int64_t now = esp_timer_get_time() / 1000;
    if (now - s_last_roll_ms < SHAKE_COOLDOWN) return;
    s_last_roll_ms = now;
    s_roll_left = ROLL_TICKS;
    s_rolled = true;
    show_result(false);
}

static void tick_cb(lv_timer_t *t)
{
    (void)t;

    /* Animación de lanzamiento */
    if (s_roll_left > 0) {
        s_roll_left--;
        bool final = (s_roll_left == 0);
        for (int d = 0; d < s_num_dice; d++) set_face(d, rand_face(), final);
        if (final) show_result(true);
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

static void mode_cb(lv_event_t *e)
{
    int n = (int)(intptr_t)lv_event_get_user_data(e);
    if (n == s_num_dice) return;
    s_num_dice = n;
    s_roll_left = 0;
    build_dice_area();
}

static void dice_open(lv_obj_t *parent)
{
    s_parent = parent;

    /* Pista / suma (debajo de los dados) */
    s_info_label = lv_label_create(parent);
    lv_obj_set_style_text_font(s_info_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_info_label, lv_color_hex(0xBFE0FF), 0);
    lv_label_set_text(s_info_label, "agita o toca");
    lv_obj_align(s_info_label, LV_ALIGN_CENTER, 0, 66);

    /* Selector 1 / 2 dados (abajo) */
    for (int i = 0; i < 2; i++) {
        s_mode_btns[i] = lv_btn_create(parent);
        lv_obj_set_size(s_mode_btns[i], 36, 26);
        lv_obj_align(s_mode_btns[i], LV_ALIGN_BOTTOM_MID, (i == 0) ? -22 : 22, -12);
        lv_obj_set_style_radius(s_mode_btns[i], 13, 0);
        lv_obj_add_event_cb(s_mode_btns[i], mode_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)(i + 1));
        lv_obj_t *lbl = lv_label_create(s_mode_btns[i]);
        lv_label_set_text(lbl, i == 0 ? "1" : "2");
        lv_obj_center(lbl);
    }

    build_dice_area();

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
    s_parent = NULL;
    s_area = NULL;
    s_info_label = NULL;
    s_mode_btns[0] = s_mode_btns[1] = NULL;
    for (int d = 0; d < MAX_DICE; d++)
        for (int i = 0; i < 9; i++) s_pips[d][i] = NULL;
}

const tool_t tool_dice = {
    .name = "Dado",
    .icon = LV_SYMBOL_SHUFFLE,
    .accent = 0xB06AFF,
    .open = dice_open,
    .close = dice_close,
};
