/*
 * ui_textinput.c — Overlay modal con textarea + teclado LVGL.
 */
#include "ui_textinput.h"

#include <string.h>

#include "lvgl.h"

static lv_obj_t *s_overlay = NULL;
static lv_obj_t *s_ta = NULL;
static ui_text_cb_t s_cb = NULL;
static void *s_user = NULL;

static void close_overlay(void)
{
    if (s_overlay) {
        lv_obj_del(s_overlay);
        s_overlay = NULL;
        s_ta = NULL;
    }
}

/* Confirma: copia el texto, cierra el overlay y llama al callback. */
static void accept(void)
{
    /* Copiar el texto antes de cerrar (el cierre destruye el textarea) */
    static char buf[128];
    lv_snprintf(buf, sizeof(buf), "%s", lv_textarea_get_text(s_ta));
    ui_text_cb_t cb = s_cb;
    void *user = s_user;
    close_overlay();
    if (cb) cb(buf, user);
}

static void kb_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        accept();
    } else if (code == LV_EVENT_CANCEL) {
        close_overlay();
    }
}

/*
 * En la pantalla redonda la tecla ✓ (esquina inferior derecha) queda cortada
 * por el círculo. La tecla enter (↵) sí es visible, así que la tratamos como
 * "aceptar" (en un campo de una línea no tiene otro uso).
 */
static void kb_value_changed_cb(lv_event_t *e)
{
    lv_obj_t *kb = lv_event_get_target(e);
    uint16_t id = lv_btnmatrix_get_selected_btn(kb);
    if (id == LV_BTNMATRIX_BTN_NONE) return;
    const char *txt = lv_btnmatrix_get_btn_text(kb, id);
    if (txt && strcmp(txt, LV_SYMBOL_NEW_LINE) == 0) {
        accept();
    }
}

static void cancel_btn_cb(lv_event_t *e)
{
    (void)e;
    close_overlay();
}

void ui_text_prompt(const char *title, const char *initial, int max_len,
                    bool password, ui_text_cb_t cb, void *user_data)
{
    if (s_overlay) return; /* ya hay un prompt abierto */

    s_cb = cb;
    s_user = user_data;

    /* Overlay a pantalla completa sobre todo lo demás */
    s_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_overlay);
    lv_obj_set_size(s_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_overlay, lv_color_hex(0x001B2E), 0);
    lv_obj_set_style_bg_opa(s_overlay, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_overlay, LV_OBJ_FLAG_SCROLLABLE);

    /* Título */
    lv_obj_t *lbl = lv_label_create(s_overlay);
    lv_label_set_text(lbl, title);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xBFE0FF), 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 12);

    /* Campo de texto */
    s_ta = lv_textarea_create(s_overlay);
    lv_obj_set_size(s_ta, 170, 36);
    lv_obj_align(s_ta, LV_ALIGN_TOP_MID, 0, 34);
    lv_textarea_set_one_line(s_ta, true);
    lv_textarea_set_max_length(s_ta, max_len);
    lv_textarea_set_password_mode(s_ta, password);
    lv_textarea_set_text(s_ta, initial ? initial : "");

    /* Botón cancelar en el hueco entre el campo y el teclado */
    lv_obj_t *cancel = lv_btn_create(s_overlay);
    lv_obj_set_size(cancel, 92, 26);
    lv_obj_align(cancel, LV_ALIGN_TOP_MID, 0, 76);
    lv_obj_set_style_radius(cancel, 13, 0);
    lv_obj_set_style_bg_color(cancel, lv_color_hex(0x5A3333), 0);
    lv_obj_add_event_cb(cancel, cancel_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *clbl = lv_label_create(cancel);
    lv_label_set_text(clbl, "Cancelar");
    lv_obj_center(clbl);

    /* Teclado levantado unos px para que la fila inferior entre al círculo;
     * la tecla ↵ (visible) actúa como aceptar — ver kb_value_changed_cb. */
    lv_obj_t *kb = lv_keyboard_create(s_overlay);
    lv_obj_set_size(kb, 240, 120);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_keyboard_set_textarea(kb, s_ta);
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb, kb_value_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
}
