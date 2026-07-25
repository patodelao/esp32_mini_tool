/*
 * tool_cam.c — Configura la captura de fotos del nodo cam.
 *
 * La captura la hace un agente en la Raspberry (cam_capture.py): baja
 * http://<cam>/foto.jpg cada X minutos y la guarda con fecha. Desde acá se lo
 * enciende/apaga y se elige el intervalo, publicando la config RETENIDA con el
 * mismo patrón que los umbrales del suelo:
 *
 *     labo/config/cam/captura/activo     "1"/"0"
 *     labo/config/cam/captura/intervalo  minutos
 *
 * El minitool es el único que escribe esta config, así que guarda su intención
 * en NVS y la re-publica al abrir la tool: si el broker se reinició y perdió el
 * retenido, con abrir esta pantalla se restablece.
 *
 * El agente devuelve cuántas fotos lleva guardadas en labo/sensor/cam/fotos,
 * que aparece solo en la tool Sensores.
 */
#include "tool.h"
#include "ui_theme.h"
#include "mqtt_hub.h"

#include "nvs.h"

#include <stdio.h>
#include <string.h>

#define TOPIC_ACTIVO    "labo/config/cam/captura/activo"
#define TOPIC_INTERVALO "labo/config/cam/captura/intervalo"

/* Opciones de intervalo, en minutos. El roller muestra estas mismas. */
static const int INTERVALOS[] = { 1, 2, 5, 10, 15, 30, 60 };
#define N_INTERVALOS ((int)(sizeof(INTERVALOS) / sizeof(INTERVALOS[0])))
#define DEFAULT_MIN 10

static lv_obj_t *s_sw = NULL;      /* encender / apagar */
static lv_obj_t *s_roller = NULL;  /* intervalo */
static lv_obj_t *s_hint = NULL;

/* --- Persistencia local (la intención del usuario) --- */

static void load(bool *activo, int *minutos)
{
    *activo = false;
    *minutos = DEFAULT_MIN;
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t a = 0, m = DEFAULT_MIN;
    if (nvs_get_u8(h, "cam_cap", &a) == ESP_OK) *activo = (a != 0);
    if (nvs_get_u8(h, "cam_int", &m) == ESP_OK) *minutos = m;
    nvs_close(h);
}

static void save(bool activo, int minutos)
{
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "cam_cap", activo ? 1 : 0);
    nvs_set_u8(h, "cam_int", (uint8_t)minutos);
    nvs_commit(h);
    nvs_close(h);
}

/* --- Publicar la config al broker (retenida) --- */

static void publish(bool activo, int minutos)
{
    char v[8];
    mqtt_hub_publish(TOPIC_ACTIVO, activo ? "1" : "0", 1, true);
    snprintf(v, sizeof(v), "%d", minutos);
    mqtt_hub_publish(TOPIC_INTERVALO, v, 1, true);
}

static int selected_minutos(void)
{
    int i = (int)lv_roller_get_selected(s_roller);
    if (i < 0 || i >= N_INTERVALOS) i = 0;
    return INTERVALOS[i];
}

static void refresh_hint(bool activo, int minutos)
{
    if (!s_hint) return;
    char buf[48];
    if (activo) snprintf(buf, sizeof(buf), "Guardando cada %d min\nen la Raspberry", minutos);
    else        snprintf(buf, sizeof(buf), "Captura apagada");
    lv_label_set_text(s_hint, buf);
    lv_obj_set_style_text_color(s_hint, lv_color_hex(activo ? UI_OK : UI_MUTED), 0);
}

/* Cualquier cambio: guarda, publica y actualiza el texto. */
static void changed_cb(lv_event_t *e)
{
    (void)e;
    bool activo = lv_obj_has_state(s_sw, LV_STATE_CHECKED);
    int minutos = selected_minutos();
    save(activo, minutos);
    publish(activo, minutos);
    refresh_hint(activo, minutos);
}

static void cam_open(lv_obj_t *parent)
{
    bool activo; int minutos;
    load(&activo, &minutos);

    ui_title(parent, "Camara");

    /* Fila: "Capturar" + switch */
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(UI_TEXT), 0);
    lv_label_set_text(lbl, "Capturar");
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, -28, 52);

    s_sw = lv_switch_create(parent);
    lv_obj_set_style_bg_color(s_sw, lv_color_hex(UI_OK), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_align(s_sw, LV_ALIGN_TOP_MID, 44, 50);
    if (activo) lv_obj_add_state(s_sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_sw, changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Intervalo: roller con los minutos */
    char opts[64] = "";
    for (int i = 0; i < N_INTERVALOS; i++) {
        char one[10];
        snprintf(one, sizeof(one), "%d min%s", INTERVALOS[i], i < N_INTERVALOS - 1 ? "\n" : "");
        strncat(opts, one, sizeof(opts) - strlen(opts) - 1);
    }
    s_roller = lv_roller_create(parent);
    lv_roller_set_options(s_roller, opts, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(s_roller, 3);
    lv_obj_set_width(s_roller, 110);
    lv_obj_align(s_roller, LV_ALIGN_CENTER, 0, 18);
    lv_obj_set_style_bg_opa(s_roller, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_roller, lv_color_hex(UI_CARD), LV_PART_SELECTED);
    lv_obj_set_style_text_color(s_roller, lv_color_hex(UI_TEXT), LV_PART_SELECTED);
    lv_obj_add_event_cb(s_roller, changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    for (int i = 0; i < N_INTERVALOS; i++) {
        if (INTERVALOS[i] == minutos) { lv_roller_set_selected(s_roller, i, LV_ANIM_OFF); break; }
    }

    s_hint = lv_label_create(parent);
    lv_obj_set_style_text_font(s_hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(s_hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_hint, LV_ALIGN_BOTTOM_MID, 0, -26);
    refresh_hint(activo, minutos);

    /* Re-asentar en el broker lo que el minitool tiene guardado: si el broker
     * perdió el retenido, con abrir esta pantalla se restablece. */
    publish(activo, minutos);
}

static void cam_close(void)
{
    s_sw = NULL;
    s_roller = NULL;
    s_hint = NULL;
}

const tool_t tool_cam = {
    .name = "Camara",
    .icon = LV_SYMBOL_IMAGE,
    .accent = 0x2ED9A3,
    .open = cam_open,
    .close = cam_close,
};
