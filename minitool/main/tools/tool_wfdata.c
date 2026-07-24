/*
 * tool_wfdata.c — Elegir qué dato muestra la carátula abajo a la izquierda.
 *
 * Ese hueco mostraba siempre los pasos del día. El mecanismo era bueno —es el
 * único lugar donde se lee un dato sin tocar nada— pero el dato no le sirve a
 * todo el mundo. Acá se elige: los pasos, o cualquier sensor del home-lab.
 *
 * La lista se arma en el momento con los sensores que estén vivos, así que un
 * nodo nuevo aparece solo. Si el sensor elegido después desaparece, la carátula
 * muestra "--" en vez de callarse: haberlo elegido y que no llegue también es
 * información.
 *
 * Vive dentro de Config (hidden), como el resto de los ajustes.
 */
#include "tool.h"
#include "ui_theme.h"
#include "ui_watchface.h"
#include "sensor_service.h"

#include <stdio.h>
#include <string.h>

#define CARD_W   180
#define CARD_H    44
#define LIST_PAD ((240 - CARD_H) / 2)

static lv_obj_t *s_list = NULL;

/* Cada fila lleva su id en el user_data (cadena estática por fila). */
static char s_ids[20][SENSOR_ID_MAX];
static lv_obj_t *s_checks[20];
static int s_rows = 0;

static void mark_selected(void)
{
    char cur[SENSOR_ID_MAX];
    ui_watchface_get_slot(cur, sizeof(cur));

    for (int i = 0; i < s_rows; i++) {
        bool on = (strcmp(s_ids[i], cur) == 0);
        lv_label_set_text(s_checks[i], on ? LV_SYMBOL_OK : "");
    }
}

static void pick_cb(lv_event_t *e)
{
    ui_watchface_set_slot((const char *)lv_event_get_user_data(e));
    mark_selected();
}

/* Lista curvada: cada fila se corre siguiendo el borde circular y se atenúa al
 * alejarse del centro. Mismo criterio que la lista de Config. */
static void apply_curve(lv_obj_t *cont)
{
    lv_area_t a;
    lv_obj_get_coords(cont, &a);
    lv_coord_t cy = (a.y1 + a.y2) / 2;

    uint32_t n = lv_obj_get_child_cnt(cont);
    for (uint32_t i = 0; i < n; i++) {
        lv_obj_t *child = lv_obj_get_child(cont, i);
        lv_area_t ca;
        lv_obj_get_coords(child, &ca);
        lv_coord_t dy = (ca.y1 + ca.y2) / 2 - cy;

        int32_t d = dy < 0 ? -dy : dy;
        if (d > 110) d = 110;
        /* x = R - sqrt(R^2 - dy^2): el borde del círculo, aproximado sin sqrt. */
        lv_coord_t dx = (lv_coord_t)((d * d) / 260);
        lv_obj_set_style_translate_x(child, dx, 0);

        lv_opa_t op = (d > 100) ? LV_OPA_20 : (lv_opa_t)(LV_OPA_COVER - d * 2);
        lv_obj_set_style_opa(child, op, 0);
    }
}

static void curve_cb(lv_event_t *e) { apply_curve(lv_event_get_target(e)); }

static void row_add(const char *label, const char *id, uint32_t chip)
{
    if (s_rows >= (int)(sizeof(s_checks) / sizeof(s_checks[0]))) return;

    strlcpy(s_ids[s_rows], id, SENSOR_ID_MAX);

    lv_obj_t *card = lv_obj_create(s_list);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, CARD_W, CARD_H);
    lv_obj_set_style_radius(card, CARD_H / 2, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD_PRESS), LV_STATE_PRESSED);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(card, pick_cb, LV_EVENT_CLICKED, (void *)s_ids[s_rows]);

    lv_obj_t *dot = lv_obj_create(card);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 10, 10);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(chip), 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_align(dot, LV_ALIGN_LEFT_MID, 14, 0);

    lv_obj_t *t = lv_label_create(card);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(UI_TEXT), 0);
    lv_label_set_long_mode(t, LV_LABEL_LONG_DOT);
    lv_obj_set_width(t, CARD_W - 66);
    lv_label_set_text(t, label);
    lv_obj_align(t, LV_ALIGN_LEFT_MID, 32, 0);

    lv_obj_t *chk = lv_label_create(card);
    lv_obj_set_style_text_color(chk, lv_color_hex(UI_OK), 0);
    lv_label_set_text(chk, "");
    lv_obj_align(chk, LV_ALIGN_RIGHT_MID, -14, 0);

    s_checks[s_rows++] = chk;
}

static void wfdata_open(lv_obj_t *parent)
{
    s_rows = 0;

    /* Sin acentos: la fuente del display no los tiene. */
    ui_title(parent, "Caratula");

    s_list = lv_obj_create(parent);
    lv_obj_remove_style_all(s_list);
    lv_obj_set_size(s_list, 240, 240);
    lv_obj_center(s_list);
    lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_list, 8, 0);
    lv_obj_set_style_pad_top(s_list, LIST_PAD, 0);
    lv_obj_set_style_pad_bottom(s_list, LIST_PAD, 0);
    lv_obj_set_scroll_dir(s_list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(s_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(s_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_list, LV_OBJ_FLAG_EVENT_BUBBLE);   /* deja salir por gesto */
    lv_obj_add_event_cb(s_list, curve_cb, LV_EVENT_SCROLL, NULL);

    row_add("Pasos del dia", WF_SLOT_STEPS, UI_OK);

    int n = sensor_count();
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16], name[40];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;
        sensor_friendly_name(id, name, sizeof(name));
        row_add(name, id, 0x4AA8FF);
    }

    mark_selected();
    lv_obj_update_layout(s_list);
    apply_curve(s_list);
}

static void wfdata_close(void)
{
    s_list = NULL;
    s_rows = 0;
}

const tool_t tool_wfdata = {
    .name = "Caratula",
    .icon = LV_SYMBOL_EYE_OPEN,
    .accent = 0x2ED9A3,
    .open = wfdata_open,
    .close = wfdata_close,
    .hidden = true,   /* vive dentro de la tool Config */
};
