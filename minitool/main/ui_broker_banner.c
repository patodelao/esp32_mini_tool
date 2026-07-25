/*
 * ui_broker_banner.c — Implementación del aviso de broker caído.
 */
#include "ui_broker_banner.h"
#include "ui_theme.h"
#include "mqtt_hub.h"
#include "wifi_manager.h"

#include "lvgl.h"

/* Segundos que el broker tiene que estar caído antes de avisar. Evita que la
 * píldora parpadee en reconexiones normales, en el arranque o en un OTA de otro
 * nodo. */
#define GRACE_S 8

static lv_obj_t   *s_pill = NULL;
static lv_timer_t *s_timer = NULL;
static int         s_down_s = 0;

static void tick(lv_timer_t *t)
{
    (void)t;
    if (!s_pill) return;

    /* Sin Wi-Fi, el problema es la red, no el broker: de eso ya avisa el icono
     * de Wi-Fi de la carátula. Acá solo interesa el caso "hay red pero la Pi
     * no está". */
    bool broker_caido = wifi_manager_is_connected() && !mqtt_hub_connected();

    if (broker_caido) {
        if (s_down_s < GRACE_S) s_down_s++;
    } else {
        s_down_s = 0;
    }

    bool mostrar = (s_down_s >= GRACE_S);
    if (mostrar) {
        lv_obj_clear_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
        /* La carátula y otras capas se crean sobre lv_layer_top; re-subir la
         * píldora la mantiene visible por encima de ellas. */
        lv_obj_move_foreground(s_pill);
    } else {
        lv_obj_add_flag(s_pill, LV_OBJ_FLAG_HIDDEN);
    }
}

void ui_broker_banner_init(void)
{
    if (s_pill) return;

    s_pill = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_pill);
    lv_obj_set_size(s_pill, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(s_pill, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_bg_color(s_pill, lv_color_hex(UI_WARN), 0);
    lv_obj_set_style_bg_opa(s_pill, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_pill, 10, 0);
    lv_obj_set_style_pad_hor(s_pill, 10, 0);
    lv_obj_set_style_pad_ver(s_pill, 3, 0);
    /* Informativo: no roba toques ni gestos a lo que haya debajo. */
    lv_obj_clear_flag(s_pill, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_pill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_pill, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *lbl = lv_label_create(s_pill);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(UI_SCREEN), 0);
    lv_label_set_text(lbl, LV_SYMBOL_WARNING " Sin broker");
    lv_obj_center(lbl);

    s_timer = lv_timer_create(tick, 1000, NULL);
}
