/*
 * ui_menu.c — Menú principal como RULETA GIRATORIA para pantalla redonda.
 *
 * Las herramientas son chips (ícono en un círculo del color de la tool) ubicados
 * ALREDEDOR del borde. Un puntero fijo arriba (el SELECTOR) marca la ranura
 * elegida, y en el centro se ve el ícono + nombre del elegido (más reloj y
 * Wi-Fi/BT), derecho y legible.
 *
 * Control: como una ruleta. Se arrastra con el dedo en redondo y el anillo gira
 * siguiendo la mano; al soltar sigue girando con INERCIA y va frenando por
 * fricción hasta hacer snap al ítem que quede bajo el puntero. Un toque (sin
 * arrastrar) abre: sobre un chip abre esa tool; en el centro abre la elegida.
 *
 * Se reubican los chips (barato, fluido) en vez de rotar la pantalla: esta placa
 * no tiene PSRAM y el pool de LVGL es chico, así que rotar el frame entero por
 * software no entra en memoria. El centro va derecho (no se rota texto).
 */
#include "ui_menu.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "esp_log.h"
#include "nvs.h"
#include "tool.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "ui_watchface.h"
#include "ui_theme.h"

static const char *TAG = "menu";

#define NVS_NS         "storage"
#define NVS_KEY_ORDER  "menu_order"

#define IDLE_TIMEOUT_MS 20000

/* Geometría. RING_R = radio (px) del CENTRO de cada chip. */
#define RING_R      92
#define HIDE_ARC    150.0f   /* chips más atrás de esto se ocultan (detrás)   */
#define SCR_CX      120      /* centro de la pantalla (240x240)               */
#define SCR_CY      120

/* --- Física de la ruleta ---------------------------------------------------- */
#define SPIN_TICK_MS   20        /* 50 Hz: paso de la inercia                  */
#define SPIN_FRICTION  0.985f    /* cuánto conserva la velocidad por tick
                                  * (más alto = gira más rato). 0..1          */
#define SPIN_MIN       28.0f     /* deg/s: por debajo frena y hace snap        */
#define SPIN_MAX       1500.0f   /* deg/s: tope de velocidad al flickear       */
#define SPIN_SIGN      (1.0f)    /* sentido de arrastre; flipear si va al revés*/
#define SNAP_LERP      0.40f     /* qué tan rápido asienta al frenar (0..1)    */
#define MOVE_THRESH    8.0f      /* px de movimiento para contar "arrastre"    */
#define CENTER_TAP_R   46.0f     /* radio del toque central = abrir la elegida */
#define FLICK_MAX_MS   120       /* si soltás >esto tras el último movimiento,
                                  * no es flick (frena y snap, no vuela)       */

#define ACCENT_DEFAULT 0x2E82C8

static lv_obj_t *s_ring = NULL;          /* backdrop + captura del toque        */
static lv_obj_t *s_cards[64];            /* un chip por tool visible            */
static lv_obj_t *s_center_icon = NULL;   /* ícono grande del elegido (centro)   */
static lv_obj_t *s_center_name = NULL;   /* nombre del elegido (centro)         */
static lv_obj_t *s_pointer = NULL;       /* SELECTOR fijo arriba                */
static lv_obj_t *s_clock_lbl = NULL;
static lv_obj_t *s_wifi_icon = NULL;
static lv_obj_t *s_bt_icon = NULL;
static const tool_t *s_current_tool = NULL;
static lv_timer_t *s_idle_timer = NULL;
static lv_timer_t *s_spin_timer = NULL;
static int s_dia = 44;                    /* diámetro de chip (para hit-test)    */

/* Mapa fila del menú -> índice en g_tools[]. Las tools 'hidden' no salen en el
 * menú principal (viven dentro de Config), así que hay que traducir. */
static int s_visible[64];
static int s_visible_count = 0;

/* Tool a la que vuelve el gesto de "atrás" (NULL = menú principal). */
static const tool_t *s_return_tool = NULL;

/* Ranura a restaurar al volver de una tool (default 0 al encender). */
static uint16_t s_return_pos = 0;

/* Orden del menú (reordenable y persistido). "identidad" = índice en s_visible
 * (fijo por tool); "rank" = posición en el anillo. s_order[rank]=identidad,
 * s_rank[identidad]=rank. Se guarda por NOMBRE de tool (robusto a cambios). */
static int s_order[64];
static int s_rank[64];

/* Modo reordenar. */
static bool  s_edit = false;      /* arrastrando para reordenar               */
static int   s_grab = -1;         /* identidad agarrada                        */
static float s_edit_accum = 0;    /* arrastre acumulado (para swaps por paso) */
static bool  s_ignore_release = false; /* ignorar el release que ENTRA a edit */
static lv_obj_t *s_hint = NULL;   /* pista en modo reordenar                  */

/* Estado de la ruleta. */
static float s_yaw = 0;          /* rotación del anillo (grados)               */
static float s_spin = 0;         /* velocidad angular por inercia (deg/s)      */
static int   s_sel_pos = -1;     /* identidad bajo el puntero (la elegida)     */
static bool  s_snapping = false; /* asentando hacia el elegido                 */
static float s_snap_to = 0;      /* objetivo de s_yaw durante el snap          */

/* Estado del arrastre. */
static bool     s_touching = false;
static bool     s_moved = false;   /* se arrastró (no fue un toque)            */
static float    s_last_ang = 0;    /* ángulo del dedo en el tick anterior      */
static float    s_press_x = 0, s_press_y = 0;
static float    s_drag_vel = 0;    /* velocidad de arrastre suavizada (deg/s)  */
static uint32_t s_last_move_ms = 0;

/* Normaliza un ángulo a (-180, 180]. */
static float wrapf(float a)
{
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

/* Ángulo (grados) del punto respecto del centro de la pantalla. */
static float angle_of(float px, float py)
{
    return atan2f(py - SCR_CY, px - SCR_CX) * 180.0f / (float)M_PI;
}

static const tool_t *tool_at(int pos)
{
    if (pos < 0 || pos >= s_visible_count) return NULL;
    return g_tools[s_visible[pos]];
}

static uint32_t pos_accent(int pos)
{
    const tool_t *t = tool_at(pos);
    return (t && t->accent) ? t->accent : ACCENT_DEFAULT;
}

static void close_current_tool(void)
{
    if (s_current_tool && s_current_tool->close) {
        s_current_tool->close();
    }
    s_current_tool = NULL;
}

static void update_statusbar(void)
{
    if (s_clock_lbl) {
        time_t now;
        struct tm ti;
        time(&now);
        localtime_r(&now, &ti);
        char buf[8];
        if (ti.tm_year >= (2020 - 1900)) {
            strftime(buf, sizeof(buf), "%H:%M", &ti);
        } else {
            snprintf(buf, sizeof(buf), "--:--");
        }
        lv_label_set_text(s_clock_lbl, buf);
    }
    if (s_wifi_icon) {
        if (wifi_manager_is_connected()) lv_obj_clear_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);
        else                             lv_obj_add_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_bt_icon) {
        bt_state_t bt = bt_manager_state();
        bool on = (bt == BT_STATE_ADVERTISING || bt == BT_STATE_CONNECTED);
        if (on) lv_obj_clear_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);
        else    lv_obj_add_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

static void idle_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_current_tool || !s_ring || ui_watchface_active()) return;
    update_statusbar();
    if (lv_disp_get_inactive_time(NULL) > IDLE_TIMEOUT_MS) {
        ui_watchface_show();
    }
}

/* Qué herramientas salen en el menú (las 'hidden' viven dentro de Config). */
static void build_visible(void)
{
    s_visible_count = 0;
    for (int i = 0; i < g_tools_count &&
         s_visible_count < (int)(sizeof(s_visible) / sizeof(s_visible[0])); i++) {
        if (g_tools[i] && g_tools[i]->hidden) continue;
        s_visible[s_visible_count++] = i;
    }
}

/* --- Orden del menú (reordenable, persistido en NVS) ----------------------- */

static const char *identity_name(int id)
{
    if (id < 0 || id >= s_visible_count) return NULL;
    const tool_t *t = g_tools[s_visible[id]];
    return t ? t->name : NULL;
}

static void order_rebuild_rank(void)
{
    for (int r = 0; r < s_visible_count; r++) s_rank[s_order[r]] = r;
}

static void order_default(void)
{
    for (int i = 0; i < s_visible_count; i++) s_order[i] = i;
    order_rebuild_rank();
}

static int identity_by_name(const char *name)
{
    if (!name) return -1;
    for (int i = 0; i < s_visible_count; i++) {
        const char *n = identity_name(i);
        if (n && strcmp(n, name) == 0) return i;
    }
    return -1;
}

/* Carga el orden guardado (lista de nombres). Las tools que no estén en el
 * guardado (nuevas) van al final en su orden default; los nombres que ya no
 * existen se ignoran. Si no hay nada guardado, orden default. */
static void order_load(void)
{
    order_default();

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    char buf[640];
    size_t sz = sizeof(buf) - 1;
    esp_err_t err = nvs_get_blob(h, NVS_KEY_ORDER, buf, &sz);
    nvs_close(h);
    if (err != ESP_OK) return;
    buf[sz] = '\0';

    bool placed[64] = { false };
    int r = 0;
    char *save = NULL;
    for (char *tok = strtok_r(buf, "\n", &save);
         tok && r < s_visible_count; tok = strtok_r(NULL, "\n", &save)) {
        int id = identity_by_name(tok);
        if (id >= 0 && !placed[id]) { s_order[r++] = id; placed[id] = true; }
    }
    for (int i = 0; i < s_visible_count && r < s_visible_count; i++) {
        if (!placed[i]) { s_order[r++] = i; placed[i] = true; }
    }
    order_rebuild_rank();
}

static void order_save(void)
{
    char buf[640];
    int off = 0;
    for (int r = 0; r < s_visible_count; r++) {
        const char *name = identity_name(s_order[r]);
        if (!name) name = "?";
        int m = snprintf(buf + off, sizeof(buf) - off, "%s\n", name);
        if (m < 0 || off + m >= (int)sizeof(buf)) break;
        off += m;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY_ORDER, buf, off);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Orden del menú guardado (%d tools)", s_visible_count);
}

/* Mueve la tool agarrada un paso (dir = +1/-1), sin dar la vuelta. Mantiene la
 * agarrada bajo el puntero (s_yaw alineado a su rank). */
static void order_move(int dir)
{
    if (s_grab < 0) return;
    int r = s_rank[s_grab];
    int r2 = r + dir;
    if (r2 < 0 || r2 >= s_visible_count) return;

    int other = s_order[r2];
    s_order[r] = other; s_order[r2] = s_grab;
    s_rank[other] = r;  s_rank[s_grab] = r2;

    s_yaw = (float)r2 * (360.0f / (float)s_visible_count);
}

/* Reubica cada chip según s_yaw, elige el de arriba y actualiza el centro y el
 * puntero. Los chips no rotan sobre sí (son círculos): solo cambian de lugar. */
static void layout_ring(void)
{
    int n = s_visible_count;
    if (!s_ring || n <= 0) return;
    float pitch = 360.0f / (float)n;

    /* 1) elegido = la identidad en el rank que cae bajo el puntero (arriba) */
    int prank = ((int)lroundf(s_yaw / pitch)) % n;
    if (prank < 0) prank += n;
    int sel = s_order[prank];

    /* 2) ubicar cada chip en su RANK; los de atrás se atenúan / se ocultan */
    for (int i = 0; i < n; i++) {
        lv_obj_t *chip = s_cards[i];
        if (!chip) continue;

        float a = wrapf((float)s_rank[i] * pitch - s_yaw);
        float aa = a < 0 ? -a : a;

        if (aa > HIDE_ARC) { lv_obj_add_flag(chip, LV_OBJ_FLAG_HIDDEN); continue; }
        lv_obj_clear_flag(chip, LV_OBJ_FLAG_HIDDEN);

        float ar = a * (float)M_PI / 180.0f;
        lv_obj_align(chip, LV_ALIGN_CENTER,
                     (lv_coord_t)(sinf(ar) * RING_R),
                     (lv_coord_t)(-cosf(ar) * RING_R));

        bool on = (i == sel);
        bool grabbed = (s_edit && i == s_grab);
        int opa = 255 - (int)(aa * 140.0f / 180.0f);
        if (opa < 70) opa = 70;
        lv_obj_set_style_opa(chip, (on || grabbed) ? LV_OPA_COVER : (lv_opa_t)opa, 0);
        lv_obj_set_style_border_width(chip, (on || grabbed) ? 3 : 0, 0);
        if (grabbed)  lv_obj_set_style_border_color(chip, lv_color_white(), 0);
        else if (on)  lv_obj_set_style_border_color(chip, lv_color_hex(pos_accent(i)), 0);
    }

    /* 3) centro + puntero con el color/nombre del elegido (solo si cambió) */
    if (sel != s_sel_pos) {
        s_sel_pos = sel;
        const tool_t *t = tool_at(sel);
        uint32_t accent = pos_accent(sel);
        if (s_center_icon) {
            lv_label_set_text(s_center_icon, (t && t->icon) ? t->icon : "");
            lv_obj_set_style_text_color(s_center_icon, lv_color_hex(accent), 0);
        }
        if (s_center_name) lv_label_set_text(s_center_name, (t && t->name) ? t->name : "");
        if (s_pointer)     lv_obj_set_style_bg_color(s_pointer, lv_color_hex(accent), 0);
    }
}

/* Inercia + snap: corre siempre, gateada al menú. Mientras se toca, manda el
 * arrastre (esta no hace nada). */
static void spin_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_current_tool || !s_ring || ui_watchface_active() || s_touching || s_edit) return;

    bool changed = false;

    if (s_spin > SPIN_MIN || s_spin < -SPIN_MIN) {
        /* Girando por inercia: avanza y frena por fricción. */
        s_yaw += s_spin * (SPIN_TICK_MS / 1000.0f);
        s_spin *= SPIN_FRICTION;
        changed = true;
        lv_disp_trig_activity(NULL);
        if (s_spin < SPIN_MIN && s_spin > -SPIN_MIN) {
            /* Se frenó: asentar en la RANURA (rank) más cercana, así queda una
             * tool justo bajo el puntero. */
            s_spin = 0;
            float pitch = 360.0f / (float)s_visible_count;
            s_snap_to = roundf(s_yaw / pitch) * pitch;
            s_snapping = true;
        }
    }

    if (s_snapping) {
        float d = s_snap_to - s_yaw;
        if (d < 0.4f && d > -0.4f) { s_yaw = fmodf(s_snap_to, 360.0f); s_snapping = false; }
        else s_yaw += d * SNAP_LERP;
        changed = true;
    }

    if (changed) layout_ring();
}

static void open_tool_ptr(const tool_t *tool)
{
    if (!tool) return;

    ESP_LOGI(TAG, "Abriendo herramienta: %s", tool->name ? tool->name : "(sin nombre)");

    lv_obj_clean(lv_scr_act());
    s_ring = NULL;
    s_center_icon = s_center_name = s_pointer = s_hint = NULL;
    s_clock_lbl = s_wifi_icon = s_bt_icon = NULL;
    s_edit = false; s_grab = -1; s_ignore_release = false;
    s_current_tool = tool;

    if (tool->open) tool->open(lv_scr_act());
}

static void open_async_cb(void *param)
{
    close_current_tool();
    lv_obj_clean(lv_scr_act());
    open_tool_ptr((const tool_t *)param);
}

void ui_menu_open_tool(const tool_t *tool)
{
    if (!tool) return;
    s_return_tool = s_current_tool;   /* volver a quien la abrió */
    lv_async_call(open_async_cb, (void *)tool);
}

/* Abre por IDENTIDAD (índice en s_visible). Guarda el RANK para volver a la
 * misma ranura del anillo. */
static void open_pos(int id)
{
    const tool_t *t = tool_at(id);
    if (!t) return;
    s_return_pos = (uint16_t)((id >= 0 && id < s_visible_count) ? s_rank[id] : 0);
    s_return_tool = NULL;
    open_tool_ptr(t);
}

static void screen_gesture_cb(lv_event_t *e)
{
    (void)e;
    if (!s_current_tool) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;

    if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
        const tool_t *back = s_return_tool;
        s_return_tool = NULL;
        close_current_tool();
        lv_obj_clean(lv_scr_act());
        if (back) open_tool_ptr(back);
        else      create_main_menu();
    }
}

/* Un toque (sin arrastrar): si cae sobre un chip, abre esa tool; si cae en el
 * centro, abre la elegida (la del puntero). */
static void open_at_point(float px, float py)
{
    float dx = px - SCR_CX, dy = py - SCR_CY;
    if (dx * dx + dy * dy < CENTER_TAP_R * CENTER_TAP_R) { open_pos(s_sel_pos); return; }

    int n = s_visible_count;
    float pitch = (n > 0) ? 360.0f / (float)n : 0.0f;
    int best = -1;
    float bestd = (float)s_dia * 0.75f;   /* umbral de acierto sobre el chip */
    bestd *= bestd;
    for (int i = 0; i < n; i++) {
        float a = wrapf((float)s_rank[i] * pitch - s_yaw);
        if ((a < 0 ? -a : a) > HIDE_ARC) continue;   /* oculto */
        float ar = a * (float)M_PI / 180.0f;
        float cx = SCR_CX + sinf(ar) * RING_R;
        float cy = SCR_CY - cosf(ar) * RING_R;
        float d = (px - cx) * (px - cx) + (py - cy) * (py - cy);
        if (d < bestd) { bestd = d; best = i; }
    }
    if (best >= 0) open_pos(best);
    else           open_pos(s_sel_pos);
}

/* Control de la ruleta: arrastre para girar, inercia al soltar, toque para abrir. */
static void wheel_touch_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    float px = (float)p.x, py = (float)p.y;

    if (code == LV_EVENT_PRESSED) {
        s_touching = true;
        s_moved = false;
        s_spin = 0;
        s_snapping = false;
        s_drag_vel = 0;
        s_edit_accum = 0;
        s_press_x = px; s_press_y = py;
        s_last_ang = angle_of(px, py);
        s_last_move_ms = lv_tick_get();
    }
    else if (code == LV_EVENT_LONG_PRESSED) {
        /* Mantener presionado sin arrastrar = agarrar la elegida para moverla. */
        if (!s_edit && !s_moved && s_sel_pos >= 0) {
            s_edit = true;
            s_grab = s_sel_pos;
            s_edit_accum = 0;
            s_ignore_release = true;     /* el release de este long-press no fija */
            s_spin = 0; s_snapping = false;
            s_yaw = (float)s_rank[s_grab] * (360.0f / (float)s_visible_count);
            if (s_hint) lv_obj_clear_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
            layout_ring();
        }
    }
    else if (code == LV_EVENT_PRESSING) {
        float ang = angle_of(px, py);
        float dang = wrapf(ang - s_last_ang);
        s_last_ang = ang;

        float mdx = px - s_press_x, mdy = py - s_press_y;
        if (mdx * mdx + mdy * mdy > MOVE_THRESH * MOVE_THRESH) s_moved = true;

        if (s_edit) {
            /* Reordenar: cada 'pitch' de arrastre mueve la agarrada un paso. */
            float pitch = 360.0f / (float)s_visible_count;
            s_edit_accum += SPIN_SIGN * dang;
            while (s_edit_accum >=  pitch) { order_move(+1); s_edit_accum -= pitch; }
            while (s_edit_accum <= -pitch) { order_move(-1); s_edit_accum += pitch; }
            layout_ring();
            lv_disp_trig_activity(NULL);
            return;
        }

        uint32_t now = lv_tick_get();
        float dt = (float)(now - s_last_move_ms) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;

        /* El anillo sigue al dedo. SPIN_SIGN por si el sentido sale invertido. */
        s_yaw -= SPIN_SIGN * dang;

        float inst = -SPIN_SIGN * dang / dt;     /* velocidad de d(s_yaw)/dt */
        s_drag_vel += (inst - s_drag_vel) * 0.5f;
        s_last_move_ms = now;

        layout_ring();
        lv_disp_trig_activity(NULL);
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        s_touching = false;

        if (s_ignore_release) { s_ignore_release = false; return; }

        if (s_edit) {
            /* Un toque (sin arrastrar) fija el nuevo orden y sale de edición. */
            if (!s_moved) {
                order_save();
                s_edit = false;
                s_grab = -1;
                if (s_hint) lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
                layout_ring();
            }
            return;   /* en edición no hay inercia ni abrir */
        }

        if (s_moved) {
            /* Fue arrastre: si el último movimiento fue reciente, sale volando
             * con inercia; si soltó tras una pausa, frena y hace snap. */
            uint32_t idle = lv_tick_get() - s_last_move_ms;
            if (idle <= FLICK_MAX_MS) {
                s_spin = s_drag_vel;
                if (s_spin > SPIN_MAX)  s_spin = SPIN_MAX;
                if (s_spin < -SPIN_MAX) s_spin = -SPIN_MAX;
            } else {
                s_spin = 0;   /* el spin_tick hará snap al soltar quieto */
            }
        } else {
            /* Fue un toque: abrir. */
            open_at_point(s_press_x, s_press_y);
        }
    }
}

/* Chip: círculo del color de la tool con su ícono. Visual, NO clickable: el
 * toque lo maneja s_ring (la ruleta) por hit-test. */
static lv_obj_t *chip_create(int pos, int dia)
{
    const tool_t *t = tool_at(pos);

    lv_obj_t *chip = lv_obj_create(s_ring);
    lv_obj_remove_style_all(chip);
    lv_obj_set_size(chip, dia, dia);
    lv_obj_set_style_radius(chip, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(chip, lv_color_hex(pos_accent(pos)), 0);
    lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
    lv_obj_set_style_border_opa(chip, LV_OPA_COVER, 0);
    lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(chip, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *ic = lv_label_create(chip);
    lv_label_set_text(ic, (t && t->icon) ? t->icon : LV_SYMBOL_DUMMY);
    lv_obj_set_style_text_color(ic, lv_color_hex(UI_BG), 0);
    lv_obj_center(ic);

    return chip;
}

void create_main_menu(void)
{
    static bool s_registered = false;
    if (!s_registered) {
        lv_obj_add_event_cb(lv_scr_act(), screen_gesture_cb, LV_EVENT_GESTURE, NULL);
        s_idle_timer = lv_timer_create(idle_tick_cb, 1000, NULL);
        s_registered = true;
    } else if (!s_idle_timer) {
        s_idle_timer = lv_timer_create(idle_tick_cb, 1000, NULL);
    }
    if (!s_spin_timer) {
        s_spin_timer = lv_timer_create(spin_tick_cb, SPIN_TICK_MS, NULL);
    }

    close_current_tool();
    lv_obj_clean(lv_scr_act());
    /* El menú es de layout fijo: la pantalla NUNCA debe scrollear. Sin esto, un
     * hijo más ancho que la pantalla (p.ej. la pista) libera una barra de scroll
     * lateral y arrastrar de costado mueve todo. */
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);

    build_visible();
    order_load();
    int n = s_visible_count;

    /* Backdrop + captura del toque de la ruleta. Clickable para recibir los
     * eventos de presión; los chips van NO clickables para que todo el toque
     * llegue acá y lo resuelva por hit-test. */
    s_ring = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_ring);
    lv_obj_set_size(s_ring, 240, 240);
    lv_obj_center(s_ring);
    lv_obj_set_style_bg_color(s_ring, lv_color_hex(UI_SCREEN), 0);
    lv_obj_set_style_bg_opa(s_ring, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_ring, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_ring, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_ring, wheel_touch_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_ring, wheel_touch_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_ring, wheel_touch_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(s_ring, wheel_touch_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_ring, wheel_touch_cb, LV_EVENT_PRESS_LOST, NULL);

    /* Chips: diámetro adaptado a cuántas tools haya para que no se encimen. */
    int dia = 44;
    if (n > 0) {
        float per = 2.0f * (float)M_PI * (float)RING_R / (float)n;
        dia = (int)(per - 4.0f);
        if (dia > 44) dia = 44;
        if (dia < 24) dia = 24;
    }
    s_dia = dia;
    for (int i = 0; i < n; i++) {
        s_cards[i] = chip_create(i, dia);
    }

    /* Columna central (derecha/legible): reloj, ícono, nombre, Wi-Fi/BT.
     * NO clickable: el toque pasa a la ruleta y se resuelve por hit-test. */
    s_clock_lbl = lv_label_create(s_ring);
    lv_obj_set_style_text_font(s_clock_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_clock_lbl, lv_color_hex(UI_TITLE), 0);
    lv_label_set_text(s_clock_lbl, "--:--");
    lv_obj_align(s_clock_lbl, LV_ALIGN_CENTER, 0, -40);

    s_center_icon = lv_label_create(s_ring);
    lv_obj_set_style_text_font(s_center_icon, &lv_font_montserrat_28, 0);
    lv_label_set_text(s_center_icon, "");
    lv_obj_align(s_center_icon, LV_ALIGN_CENTER, 0, -6);

    s_center_name = lv_label_create(s_ring);
    lv_obj_set_style_text_font(s_center_name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_center_name, lv_color_hex(UI_TEXT), 0);
    lv_label_set_text(s_center_name, "");
    lv_obj_align(s_center_name, LV_ALIGN_CENTER, 0, 20);

    s_wifi_icon = lv_label_create(s_ring);
    lv_label_set_text(s_wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_wifi_icon, lv_color_hex(UI_TITLE), 0);
    lv_obj_align(s_wifi_icon, LV_ALIGN_CENTER, -16, 44);
    lv_obj_add_flag(s_wifi_icon, LV_OBJ_FLAG_HIDDEN);

    s_bt_icon = lv_label_create(s_ring);
    lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(s_bt_icon, lv_color_hex(UI_TITLE), 0);
    lv_obj_align(s_bt_icon, LV_ALIGN_CENTER, 16, 44);
    lv_obj_add_flag(s_bt_icon, LV_OBJ_FLAG_HIDDEN);

    /* Puntero (SELECTOR) fijo arriba: marca la ranura elegida (toma su color). */
    s_pointer = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(s_pointer);
    lv_obj_set_size(s_pointer, 12, 12);
    lv_obj_align(s_pointer, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_radius(s_pointer, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_pointer, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_pointer, LV_OBJ_FLAG_CLICKABLE);

    /* Pista del modo reordenar (oculta hasta entrar con long-press). Texto corto
     * para que entre en el ancho de la pantalla. */
    s_hint = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(s_hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_hint, lv_color_hex(UI_MUTED), 0);
    lv_label_set_text(s_hint, LV_SYMBOL_SHUFFLE "  toca para fijar");
    lv_obj_align(s_hint, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);

    /* Arrancar con la última tool abierta bajo el puntero (home_ret - yaw = 0). */
    float pitch = (n > 0) ? 360.0f / (float)n : 0.0f;
    s_yaw = (s_return_pos < (uint16_t)n) ? (float)s_return_pos * pitch : 0.0f;
    s_spin = 0;
    s_snapping = false;
    s_touching = false;
    s_edit = false;
    s_grab = -1;
    s_ignore_release = false;
    s_sel_pos = -1;      /* fuerza que layout_ring resalte y llene el centro */
    layout_ring();

    update_statusbar();
}

void ui_menu_show(void)
{
    create_main_menu();
}
