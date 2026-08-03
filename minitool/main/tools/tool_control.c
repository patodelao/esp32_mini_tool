/*
 * tool_control.c — Control remoto de tu home-lab por MQTT.
 *
 * Cada fila publica un comando en un topic. Un listener en tus ESP o en la
 * Raspberry Pi reacciona (encender relé, reset, disparar un script SSH, etc.).
 * Edita la tabla s_cmds[] con tus topics/payloads, ícono y color.
 *
 * Publica y confirma con una notificación flotante; no espera respuesta (es
 * "fire and forget": no muestra estado real del actuador salvo que el actuador
 * publique uno, cosa que hoy no hace).
 */
#include "tool.h"
#include "ui_theme.h"
#include "mqtt_hub.h"
#include "ui_notify.h"

#include <stdint.h>

typedef struct {
    const char *label;    /* texto de la fila            */
    const char *topic;    /* topic MQTT a publicar       */
    const char *payload;  /* carga útil                  */
    bool        retain;   /* dejarlo puesto en el broker */
    const char *icon;     /* ícono del chip              */
    uint32_t    accent;   /* color del chip              */
} cmd_t;

/* --- EDITA AQUÍ tus comandos --- */
static const cmd_t s_cmds[] = {
    { "Luz ON",       "labo/actuador/luz/cmd", "ON",      false, LV_SYMBOL_POWER,    0x35D07F },
    { "Luz OFF",      "labo/actuador/luz/cmd", "OFF",     false, LV_SYMBOL_POWER,    0xE74C3C },
    { "Leer pieza",   "labo/nodo/pieza/cmd",   "leer",    false, LV_SYMBOL_EYE_OPEN, 0x3498DB },
    { "Reset pieza",  "labo/nodo/pieza/cmd",   "reset",   false, LV_SYMBOL_REFRESH,  0xE0A030 },
    { "Cal suelo ON", "labo/nodo/pieza/cmd",   "cal on",  false, LV_SYMBOL_SETTINGS, 0x9B59B6 },
    { "Cal suelo OFF","labo/nodo/pieza/cmd",   "cal off", false, LV_SYMBOL_SETTINGS, 0x7D6BA8 },
    { "Leer refri",   "labo/nodo/refri/cmd",   "leer",    false, LV_SYMBOL_EYE_OPEN, 0x3498DB },
    { "Reset refri",  "labo/nodo/refri/cmd",   "reset",   false, LV_SYMBOL_REFRESH,  0xE0A030 },
    { "Backup Pi",    "labo/pi/cmd",           "backup",  false, LV_SYMBOL_SAVE,     0x1ABC9C },
};
#define CMD_COUNT (sizeof(s_cmds) / sizeof(s_cmds[0]))

static void cmd_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= (int)CMD_COUNT) return;

    if (!mqtt_hub_connected()) {
        ui_notify_push("Control", NOTIFY_WARNING, "Sin conexión MQTT");
        return;
    }
    mqtt_hub_publish(s_cmds[idx].topic, s_cmds[idx].payload, 1, s_cmds[idx].retain);
    ui_notify_push("Control", NOTIFY_INFO, s_cmds[idx].label);
}

/* Píldora con chip de ícono en color + etiqueta, mismo lenguaje que el menú. */
static void add_cmd_row(lv_obj_t *list, int idx)
{
    lv_obj_t *btn = lv_btn_create(list);
    lv_obj_remove_style_all(btn);
    lv_obj_set_size(btn, 200, 42);
    lv_obj_set_style_radius(btn, 21, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(UI_CARD_PRESS), LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, cmd_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)idx);

    lv_obj_t *chip = lv_obj_create(btn);
    lv_obj_remove_style_all(chip);
    lv_obj_set_size(chip, 30, 30);
    lv_obj_align(chip, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_radius(chip, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(chip, lv_color_hex(s_cmds[idx].accent), 0);
    lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
    lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ic = lv_label_create(chip);
    lv_label_set_text(ic, s_cmds[idx].icon);
    lv_obj_set_style_text_color(ic, lv_color_hex(UI_BG), 0);
    lv_obj_center(ic);

    lv_obj_t *name = lv_label_create(btn);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(name, lv_color_hex(UI_TEXT), 0);
    lv_label_set_text(name, s_cmds[idx].label);
    lv_obj_align(name, LV_ALIGN_LEFT_MID, 44, 0);
}

static void control_open(lv_obj_t *parent)
{
    ui_title(parent, "Control");

    lv_obj_t *list = lv_obj_create(parent);
    lv_obj_remove_style_all(list);
    lv_obj_set_size(list, 216, 172);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 14);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(list, 7, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);

    for (int i = 0; i < (int)CMD_COUNT; i++) add_cmd_row(list, i);
}

static void control_close(void) { }

const tool_t tool_control = {
    .name = "Control",
    .icon = LV_SYMBOL_UPLOAD,
    .accent = 0xE67E22,
    .open = control_open,
    .close = control_close,
};
