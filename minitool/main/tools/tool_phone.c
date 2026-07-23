/*
 * tool_phone.c — Manejar el teléfono desde el reloj (vía Gadgetbridge).
 *
 * El mismo canal BLE por el que llegan las notificaciones sirve en el otro
 * sentido. Acá se usa para lo que uno realmente quiere hacer sin sacar el
 * teléfono del bolsillo:
 *
 *   - control del reproductor (anterior / play-pausa / siguiente), que actúa
 *     sobre la app que esté sonando, sea cual sea;
 *   - "encontrar mi teléfono", que lo hace sonar aunque esté en silencio.
 *
 * Arriba se muestra qué está sonando, que Gadgetbridge manda solo al cambiar
 * de tema o de estado.
 *
 * Sin teléfono conectado los botones no hacen nada: en vez de fallar en
 * silencio, la pantalla lo dice.
 */
#include "tool.h"
#include "ble_notify.h"
#include "bt_manager.h"

#include <stdio.h>
#include <string.h>

static lv_obj_t *s_track_lbl = NULL;
static lv_obj_t *s_artist_lbl = NULL;
static lv_obj_t *s_state_lbl = NULL;
static lv_obj_t *s_play_lbl = NULL;
static lv_obj_t *s_find_btn = NULL;
static lv_timer_t *s_poll = NULL;
static bool s_playing = false;
static bool s_finding = false;

static bool phone_linked(void)
{
    return bt_manager_state() == BT_STATE_CONNECTED;
}

static void refresh(void)
{
    char track[48] = "", artist[48] = "";
    bool playing = false;
    bool have = ble_notify_music_get(track, sizeof(track), artist, sizeof(artist), &playing);
    s_playing = playing;

    if (s_track_lbl) {
        lv_label_set_text(s_track_lbl, have && track[0] ? track : "-");
    }
    if (s_artist_lbl) {
        lv_label_set_text(s_artist_lbl, have ? artist : "");
    }
    if (s_play_lbl) {
        lv_label_set_text(s_play_lbl, playing ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
    }
    if (s_state_lbl) {
        if (!phone_linked()) {
            lv_label_set_text(s_state_lbl, "Sin telefono");
            lv_obj_set_style_text_color(s_state_lbl, lv_color_hex(0xE0A030), 0);
        } else {
            lv_label_set_text(s_state_lbl, "Telefono");
            lv_obj_set_style_text_color(s_state_lbl, lv_color_hex(0x35D07F), 0);
        }
    }
}

static void poll_cb(lv_timer_t *t) { (void)t; refresh(); }

/* --------------------------------- Acciones ------------------------------- */

static void music_cb(lv_event_t *e)
{
    const char *cmd = (const char *)lv_event_get_user_data(e);
    ble_notify_music_cmd(cmd);
    /* El estado real llega cuando el teléfono responde con "musicstate";
     * mientras tanto se refleja el cambio para que el botón no se sienta
     * muerto. */
    if (strcmp(cmd, "play") == 0 || strcmp(cmd, "pause") == 0) {
        s_playing = !s_playing;
        if (s_play_lbl) lv_label_set_text(s_play_lbl, s_playing ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
    }
}

static void play_cb(lv_event_t *e)
{
    (void)e;
    ble_notify_music_cmd(s_playing ? "pause" : "play");
    s_playing = !s_playing;
    if (s_play_lbl) lv_label_set_text(s_play_lbl, s_playing ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
}

static void find_cb(lv_event_t *e)
{
    (void)e;
    s_finding = !s_finding;
    ble_notify_find_phone(s_finding);
    if (s_find_btn) {
        lv_obj_set_style_bg_color(s_find_btn,
            lv_color_hex(s_finding ? 0xB0403A : 0x33445A), 0);
    }
}

/* ----------------------------------- UI ----------------------------------- */

static lv_obj_t *round_btn(lv_obj_t *parent, int x, int size, const char *sym,
                           lv_event_cb_t cb, void *ud, lv_obj_t **out_lbl)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, size, size);
    lv_obj_align(b, LV_ALIGN_CENTER, x, 26);
    lv_obj_set_style_radius(b, size / 2, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_label_set_text(l, sym);
    lv_obj_center(l);
    if (out_lbl) *out_lbl = l;
    return b;
}

static void phone_open(lv_obj_t *parent)
{
    s_state_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_state_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_text(s_state_lbl, "");
    lv_obj_align(s_state_lbl, LV_ALIGN_TOP_MID, 0, 22);

    s_track_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_track_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_track_lbl, lv_color_white(), 0);
    lv_label_set_long_mode(s_track_lbl, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_track_lbl, 180);
    lv_obj_set_style_text_align(s_track_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_track_lbl, "-");
    lv_obj_align(s_track_lbl, LV_ALIGN_CENTER, 0, -52);

    s_artist_lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(s_artist_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_artist_lbl, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_long_mode(s_artist_lbl, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_artist_lbl, 170);
    lv_obj_set_style_text_align(s_artist_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_artist_lbl, "");
    lv_obj_align(s_artist_lbl, LV_ALIGN_CENTER, 0, -30);

    /* Transporte: el de play/pausa más grande, que es el que más se usa. */
    round_btn(parent, -62, 46, LV_SYMBOL_PREV, music_cb, (void *)"previous", NULL);
    round_btn(parent,   0, 60, LV_SYMBOL_PLAY, play_cb,  NULL, &s_play_lbl);
    round_btn(parent,  62, 46, LV_SYMBOL_NEXT, music_cb, (void *)"next", NULL);

    s_find_btn = lv_btn_create(parent);
    lv_obj_set_size(s_find_btn, 132, 34);
    lv_obj_align(s_find_btn, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_radius(s_find_btn, 17, 0);
    lv_obj_set_style_bg_color(s_find_btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(s_find_btn, find_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *fl = lv_label_create(s_find_btn);
    lv_obj_set_style_text_font(fl, &lv_font_montserrat_14, 0);
    lv_label_set_text(fl, LV_SYMBOL_BELL "  Buscar");
    lv_obj_center(fl);

    s_finding = false;
    refresh();
    s_poll = lv_timer_create(poll_cb, 700, NULL);
}

static void phone_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    /* Si quedó sonando el "buscar", callarlo al salir. */
    if (s_finding) {
        ble_notify_find_phone(false);
        s_finding = false;
    }
    s_track_lbl = s_artist_lbl = s_state_lbl = s_play_lbl = s_find_btn = NULL;
}

const tool_t tool_phone = {
    .name = "Telefono",
    .icon = LV_SYMBOL_BLUETOOTH,
    .accent = 0x3E7BFF,
    .open = phone_open,
    .close = phone_close,
};
