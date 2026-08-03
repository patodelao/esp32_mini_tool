/*
 * tool_fleet.c — Panel de nodos (ESP/equipos) del home-lab.
 *
 * Muestra los nodos vistos por fleet_service como una lista de píldoras, con el
 * mismo lenguaje visual que Sensores: un punto de estado (verde online / rojo
 * offline / gris sin señal), el nombre del nodo, y a la derecha la IP si está
 * online (que es la que se usa para OTA) o "hace X" si está caído. Solo lectura.
 *
 * Las filas se refrescan EN EL LUGAR (no se reconstruyen salvo que cambie la
 * cantidad de nodos), para no resetear el scroll mientras se mira.
 */
#include "tool.h"
#include "ui_theme.h"
#include "fleet_service.h"

#include <stdio.h>
#include <string.h>

/* Sin NINGÚN dato por más de esto => "sin señal". Alineado con FLEET_LIVENESS_S:
 * fleet_service alimenta la antigüedad con cada sensor recibido, así que un nodo
 * vivo casi nunca la alcanza. 180 s tolera un par de ciclos de telemetría
 * perdidos por un parpadeo de WiFi sin marcarlo caído. */
#define STALE_S 180
#define FLEET_ROWS_MAX 16

static lv_obj_t *s_cont = NULL;
static lv_obj_t *s_empty = NULL;
static lv_timer_t *s_poll = NULL;
static int s_built_n = -1;      /* cantidad con la que se armó (para rebuild) */

static lv_obj_t *s_dot[FLEET_ROWS_MAX];    /* punto de estado de cada fila     */
static lv_obj_t *s_right[FLEET_ROWS_MAX];  /* label derecho (IP / estado)      */
static int       s_node[FLEET_ROWS_MAX];   /* índice de nodo de cada fila      */
static int       s_rows = 0;

static void fmt_age(uint32_t s, char *out, int sz)
{
    if (s < 60)        snprintf(out, sz, "hace %us", (unsigned)s);
    else if (s < 3600) snprintf(out, sz, "hace %umin", (unsigned)(s / 60));
    else               snprintf(out, sz, "hace %uh", (unsigned)(s / 3600));
}

/* Nombre lindo: capitaliza la primera letra del id ("pieza" -> "Pieza"). */
static void nice_name(const char *id, char *out, int sz)
{
    snprintf(out, sz, "%s", id);
    if (out[0] >= 'a' && out[0] <= 'z') out[0] = (char)(out[0] - 32);
}

/* Refresca el estado/valor de una fila en el lugar. */
static void refresh_row(int r)
{
    int i = s_node[r];
    char id[24], ip[16];
    bool online;
    uint32_t age;
    if (!fleet_get(i, id, sizeof(id), &online, &age)) return;
    if (!fleet_get_ip(i, ip, sizeof(ip))) ip[0] = '\0';

    lv_color_t dc, rc;
    const char *rtext;
    char buf[24];

    if (age >= STALE_S) {
        dc = lv_color_hex(0x5A6B7A);
        fmt_age(age, buf, sizeof(buf));
        rtext = buf;
        rc = lv_color_hex(0x7F8C8D);
    } else if (online) {
        dc = lv_color_hex(0x35D07F);
        if (ip[0]) { rtext = ip;       rc = lv_color_hex(0x8FA8C8); }
        else       { rtext = "online"; rc = lv_color_hex(0x35D07F); }
    } else {
        dc = lv_color_hex(0xE74C3C);
        rtext = "offline";
        rc = lv_color_hex(0xE74C3C);
    }

    if (s_dot[r])   lv_obj_set_style_bg_color(s_dot[r], dc, 0);
    if (s_right[r]) {
        lv_label_set_text(s_right[r], rtext);
        lv_obj_set_style_text_color(s_right[r], rc, 0);
    }
}

static void build_row(int node_idx, const char *id)
{
    if (s_rows >= FLEET_ROWS_MAX) return;

    lv_obj_t *pill = lv_obj_create(s_cont);
    lv_obj_remove_style_all(pill);
    lv_obj_set_size(pill, 200, 38);
    lv_obj_set_style_radius(pill, 19, 0);
    lv_obj_set_style_bg_color(pill, lv_color_hex(UI_CARD), 0);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, 0);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dot = lv_obj_create(pill);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 12, 12);
    lv_obj_align(dot, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(0x35D07F), 0);

    char nm[24];
    nice_name(id, nm, sizeof(nm));
    lv_obj_t *name = lv_label_create(pill);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(name, lv_color_hex(0xDDE6F0), 0);
    lv_label_set_text(name, nm);
    lv_obj_align(name, LV_ALIGN_LEFT_MID, 26, 0);

    lv_obj_t *right = lv_label_create(pill);
    lv_obj_set_style_text_font(right, &lv_font_montserrat_14, 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, -12, 0);

    s_dot[s_rows]   = dot;
    s_right[s_rows] = right;
    s_node[s_rows]  = node_idx;
    s_rows++;
}

static void rebuild(void)
{
    if (!s_cont) return;
    lv_obj_clean(s_cont);
    s_rows = 0;

    int n = fleet_count();
    for (int i = 0; i < n; i++) {
        char id[24];
        bool online;
        uint32_t age;
        if (!fleet_get(i, id, sizeof(id), &online, &age)) continue;
        build_row(i, id);
    }
    for (int r = 0; r < s_rows; r++) refresh_row(r);

    s_built_n = n;
    if (s_empty) {
        if (n == 0) lv_obj_clear_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
        else        lv_obj_add_flag(s_empty, LV_OBJ_FLAG_HIDDEN);
    }
}

static void poll_cb(lv_timer_t *t)
{
    (void)t;
    if (fleet_count() != s_built_n) {
        rebuild();                       /* cambió la cantidad de nodos */
    } else {
        for (int r = 0; r < s_rows; r++) refresh_row(r);
    }
}

static void fleet_open(lv_obj_t *parent)
{
    ui_title(parent, "Nodos");

    s_cont = lv_obj_create(parent);
    lv_obj_remove_style_all(s_cont);
    lv_obj_set_size(s_cont, 216, 168);
    lv_obj_align(s_cont, LV_ALIGN_CENTER, 0, 16);
    lv_obj_set_flex_flow(s_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_cont, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_cont, 7, 0);
    lv_obj_set_scroll_dir(s_cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_cont, LV_SCROLLBAR_MODE_OFF);

    s_empty = lv_label_create(parent);
    lv_obj_set_style_text_font(s_empty, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_empty, lv_color_hex(0x7F8C8D), 0);
    lv_label_set_text(s_empty, "Sin nodos\nlabo/nodo/<id>/status");
    lv_obj_set_style_text_align(s_empty, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(s_empty);

    s_built_n = -1;
    rebuild();
    s_poll = lv_timer_create(poll_cb, 1000, NULL);
}

static void fleet_close(void)
{
    if (s_poll) { lv_timer_del(s_poll); s_poll = NULL; }
    s_cont = NULL;
    s_empty = NULL;
    s_rows = 0;
    s_built_n = -1;
}

const tool_t tool_fleet = {
    .name = "Nodos",
    .icon = LV_SYMBOL_GPS,
    .accent = 0x1ABC9C,
    .open = fleet_open,
    .close = fleet_close,
};
