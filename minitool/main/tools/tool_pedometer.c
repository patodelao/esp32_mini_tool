/*
 * tool_pedometer.c — Vista del contador de pasos.
 *
 * El conteo real vive en pedometer_service (lv_timer global, siempre activo).
 * Esta tool muestra los pasos de HOY, el progreso hacia la meta, la distancia
 * estimada y una barrita por cada uno de los últimos días, para ver la semana
 * de un vistazo en vez de un número suelto.
 */
#include "tool.h"
#include "pedometer_service.h"

#include <stdio.h>

#define STEP_METERS   0.72f   /* longitud de zancada estimada (m) */

static lv_obj_t *s_arc = NULL;
static lv_obj_t *s_count = NULL;
static lv_obj_t *s_dist = NULL;
static lv_obj_t *s_bars[PEDOMETER_DAYS];
static lv_timer_t *s_poll = NULL;
static uint32_t s_last = 0xFFFFFFFF;

/* Barras de los días cerrados, escaladas al mayor de la serie (o a la meta,
 * lo que sea más grande) para que se comparen entre sí. */
static void render_history(void)
{
    uint32_t hist[PEDOMETER_DAYS];
    int n = pedometer_history(hist, PEDOMETER_DAYS);

    uint32_t top = pedometer_get_goal();
    for (int i = 0; i < n; i++) if (hist[i] > top) top = hist[i];
    if (top == 0) top = 1;

    for (int i = 0; i < PEDOMETER_DAYS; i++) {
        if (!s_bars[i]) continue;
        if (i >= n) {                       /* día sin datos todavía */
            lv_obj_set_height(s_bars[i], 2);
            lv_obj_set_style_bg_color(s_bars[i], lv_color_hex(0x2A3A48), 0);
            continue;
        }
        int h = (int)((hist[i] * 22) / top);
        if (h < 2) h = 2;
        lv_obj_set_height(s_bars[i], h);
        /* Verde si ese día se llegó a la meta, gris si no. */
        bool hit = (pedometer_get_goal() > 0 && hist[i] >= pedometer_get_goal());
        lv_obj_set_style_bg_color(s_bars[i], lv_color_hex(hit ? 0x35D07F : 0x4A6070), 0);
    }
}

static void render(uint32_t steps)
{
    char buf[24];

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)steps);
    lv_label_set_text(s_count, buf);

    float meters = steps * STEP_METERS;
    if (meters >= 1000.0f)
        snprintf(buf, sizeof(buf), "%.2f km", (double)(meters / 1000.0f));
    else
        snprintf(buf, sizeof(buf), "%d m", (int)meters);
    lv_label_set_text(s_dist, buf);

    uint32_t goal = pedometer_get_goal();
    if (goal == 0) goal = 1;
    lv_arc_set_range(s_arc, 0, (int32_t)goal);
    lv_arc_set_value(s_arc, (int32_t)(steps > goal ? goal : steps));

    render_history();
}

static void poll_cb(lv_timer_t *t)
{
    (void)t;
    uint32_t s = pedometer_steps();
    if (s != s_last) {
        s_last = s;
        render(s);
    }
}

static void reset_cb(lv_event_t *e)
{
    (void)e;
    pedometer_reset();
    s_last = 0xFFFFFFFF; /* forzar refresco */
}

static void pedometer_open(lv_obj_t *parent)
{
    /* Arco de progreso hacia la meta, pegado al borde */
    s_arc = lv_arc_create(parent);
    lv_obj_set_size(s_arc, 232, 232);
    lv_obj_center(s_arc);
    lv_arc_set_rotation(s_arc, 270);
    lv_arc_set_bg_angles(s_arc, 0, 360);
    lv_arc_set_range(s_arc, 0, (int32_t)(pedometer_get_goal() ? pedometer_get_goal() : 1));
    lv_arc_set_value(s_arc, 0);
    lv_obj_remove_style(s_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_arc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc, lv_color_hex(0x1B2A1E), LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_arc, lv_color_hex(0x35D07F), LV_PART_INDICATOR);

    /* Título */
    lv_obj_t *title = lv_label_create(parent);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(title, "Pasos hoy");
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -50);

    /* Contador grande */
    s_count = lv_label_create(parent);
    lv_obj_set_style_text_font(s_count, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_count, lv_color_white(), 0);
    lv_label_set_text(s_count, "0");
    lv_obj_align(s_count, LV_ALIGN_CENTER, 0, -8);

    /* Distancia */
    s_dist = lv_label_create(parent);
    lv_obj_set_style_text_font(s_dist, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_dist, lv_color_hex(0x35D07F), 0);
    lv_label_set_text(s_dist, "0 m");
    lv_obj_align(s_dist, LV_ALIGN_CENTER, 0, 30);

    /* Semana: una barrita por día cerrado, alineadas por la base. */
    lv_obj_t *week = lv_obj_create(parent);
    lv_obj_remove_style_all(week);
    lv_obj_set_size(week, 112, 26);
    lv_obj_align(week, LV_ALIGN_CENTER, 0, 62);
    lv_obj_set_flex_flow(week, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(week, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_style_pad_column(week, 5, 0);
    lv_obj_clear_flag(week, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < PEDOMETER_DAYS; i++) {
        s_bars[i] = lv_obj_create(week);
        lv_obj_remove_style_all(s_bars[i]);
        lv_obj_set_width(s_bars[i], 9);
        lv_obj_set_height(s_bars[i], 2);
        lv_obj_set_style_radius(s_bars[i], 2, 0);
        lv_obj_set_style_bg_opa(s_bars[i], LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(s_bars[i], lv_color_hex(0x2A3A48), 0);
        lv_obj_clear_flag(s_bars[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    /* Botón de reset */
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 104, 32);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_radius(btn, 16, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(btn, reset_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, LV_SYMBOL_REFRESH " Reset");
    lv_obj_center(bl);

    s_last = 0xFFFFFFFF;
    render(pedometer_steps());
    s_last = pedometer_steps();
    s_poll = lv_timer_create(poll_cb, 300, NULL);
}

static void pedometer_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    s_arc = NULL;
    s_count = NULL;
    s_dist = NULL;
    for (int i = 0; i < PEDOMETER_DAYS; i++) s_bars[i] = NULL;
}

const tool_t tool_pedometer = {
    .name = "Pasos",
    .icon = LV_SYMBOL_LOOP,
    .accent = 0x35D07F,
    .open = pedometer_open,
    .close = pedometer_close,
};
