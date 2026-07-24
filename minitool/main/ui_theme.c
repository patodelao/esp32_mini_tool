/*
 * ui_theme.c — Implementación de las piezas visuales compartidas.
 */
#include "ui_theme.h"

lv_obj_t *ui_title(lv_obj_t *parent, const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(UI_TITLE), 0);
    lv_label_set_text(l, text);
    /* 18 px desde arriba: más arriba lo recorta el círculo, más abajo se come
     * el espacio del contenido. */
    lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 18);
    return l;
}

lv_obj_t *ui_card(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
                  lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, w, h);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, h / 2, 0);   /* píldora */
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    /* Que los gestos lleguen al padre: si no, una lista que ocupa la pantalla
     * se queda con el gesto de volver y la tool no tiene salida. */
    lv_obj_add_flag(card, LV_OBJ_FLAG_EVENT_BUBBLE);

    if (cb) {
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(card, lv_color_hex(UI_CARD_PRESS), LV_STATE_PRESSED);
        lv_obj_add_event_cb(card, cb, LV_EVENT_CLICKED, user_data);
    }
    return card;
}

lv_obj_t *ui_pill(lv_obj_t *parent, const char *text, uint32_t color,
                  lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, 120, 38);
    lv_obj_set_style_radius(b, 19, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, user_data);

    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    /* Sobre un color fuerte el texto va oscuro; sobre uno apagado, claro. */
    bool light_bg = (color == UI_OK) || (color == UI_WARN);
    lv_obj_set_style_text_color(l, lv_color_hex(light_bg ? UI_BG : UI_TEXT), 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}
