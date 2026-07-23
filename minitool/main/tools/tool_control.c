/*
 * tool_control.c — Control remoto de tu home-lab por MQTT.
 *
 * Cada botón publica un comando en un topic. Un pequeño listener en tus ESP o
 * en la Raspberry Pi reacciona (encender relé, reset, disparar un script SSH,
 * etc.). Edita la tabla s_cmds[] con tus topics/payloads.
 *
 * Publica y confirma con una notificación flotante; no espera respuesta.
 */
#include "tool.h"
#include "mqtt_hub.h"
#include "ui_notify.h"

#include <stdint.h>

typedef struct {
    const char *label;    /* texto del botón            */
    const char *topic;    /* topic MQTT a publicar      */
    const char *payload;  /* carga útil                 */
    bool        retain;   /* dejarlo puesto en el broker */
} cmd_t;

/* --- EDITA AQUÍ tus comandos --- */
static const cmd_t s_cmds[] = {
    { "Luz ON",      "labo/actuador/luz/cmd", "ON"      },
    { "Luz OFF",     "labo/actuador/luz/cmd", "OFF"     },
    { "Leer pieza",  "labo/nodo/pieza/cmd",   "leer"    },
    { "Reset pieza", "labo/nodo/pieza/cmd",   "reset"   },
    { "Cal suelo ON","labo/nodo/pieza/cmd",   "cal on"  },
    { "Cal suelo OFF","labo/nodo/pieza/cmd",  "cal off" },
    { "Leer refri",  "labo/nodo/refri/cmd",   "leer"    },
    { "Reset refri", "labo/nodo/refri/cmd",   "reset"   },
    /* Retenido: el refri duerme casi todo el tiempo, asi que el pedido tiene
     * que quedar puesto hasta que despierte y lo encuentre. */
    { "OTA refri",   "labo/config/refri/ota", "1",       true },
    { "Backup Pi",   "labo/pi/cmd",           "backup"  },
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

static void control_open(lv_obj_t *parent)
{
    lv_obj_t *title = lv_label_create(parent);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(title, "Control");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *list = lv_list_create(parent);
    lv_obj_set_size(list, 200, 160);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 14);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_row(list, 6, 0);

    for (int i = 0; i < (int)CMD_COUNT; i++) {
        lv_obj_t *btn = lv_list_add_btn(list, LV_SYMBOL_UPLOAD, s_cmds[i].label);
        lv_obj_set_style_radius(btn, 12, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x22303F), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_add_event_cb(btn, cmd_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

static void control_close(void) { }

const tool_t tool_control = {
    .name = "Control",
    .icon = LV_SYMBOL_UPLOAD,
    .accent = 0xE67E22,
    .open = control_open,
    .close = control_close,
};
