/*
 * tool_menuorder.c — Reordenar el menú en tiempo real.
 *
 * Muestra las herramientas en su orden actual (roller) y permite subir/bajar
 * la seleccionada. Cada movimiento persiste en NVS (vía menu_order) y se
 * refleja de inmediato aquí; al volver, el menú principal ya aparece reordenado
 * sin recompilar ni reflashear.
 */
#include "tool.h"
#include "menu_order.h"

#include "lvgl.h"

#define OPTS_BUF 768

static lv_obj_t *s_roller = NULL;
static char s_opts[OPTS_BUF];

static void build_opts(void)
{
    size_t used = 0;
    s_opts[0] = '\0';
    int count = menu_order_count();
    for (int i = 0; i < count; i++) {
        const tool_t *t = menu_order_get(i);
        const char *icon = (t && t->icon) ? t->icon : LV_SYMBOL_DUMMY;
        const char *name = (t && t->name) ? t->name : "Tool";
        const char *sep = (i == count - 1) ? "" : "\n";
        int w = lv_snprintf(s_opts + used, OPTS_BUF - used, "%s  %s%s", icon, name, sep);
        if (w <= 0 || (size_t)w >= (OPTS_BUF - used)) break;
        used += (size_t)w;
    }
}

static void refresh_roller(uint16_t select)
{
    build_opts();
    lv_roller_set_options(s_roller, s_opts, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(s_roller, select, LV_ANIM_ON);
}

static void move_dir(int dir)
{
    if (!s_roller) return;
    uint16_t sel = lv_roller_get_selected(s_roller);
    int np = menu_order_move((int)sel, dir);
    refresh_roller((uint16_t)np);
}

static void up_cb(lv_event_t *e)   { (void)e; move_dir(-1); }
static void down_cb(lv_event_t *e) { (void)e; move_dir(+1); }

static lv_obj_t *arrow_btn(lv_obj_t *parent, const char *sym, lv_align_t align, lv_coord_t dx,
                           lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 54, 44);
    lv_obj_align(btn, align, dx, -14);
    lv_obj_set_style_radius(btn, 14, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x33445A), 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = lv_label_create(btn);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
    lv_label_set_text(l, sym);
    lv_obj_center(l);
    return btn;
}

static void menuorder_open(lv_obj_t *parent)
{
    lv_obj_t *title = lv_label_create(parent);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x8FA8C8), 0);
    lv_label_set_text(title, "Ordenar menu");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

    s_roller = lv_roller_create(parent);
    build_opts();
    lv_roller_set_options(s_roller, s_opts, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(s_roller, 3);
    lv_obj_set_width(s_roller, 200);
    lv_obj_align(s_roller, LV_ALIGN_CENTER, 0, -6);

    lv_obj_set_style_bg_opa(s_roller, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_roller, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_roller, 18, LV_PART_SELECTED);
    lv_obj_set_style_bg_color(s_roller, lv_color_hex(0x2E82C8), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(s_roller, LV_OPA_30, LV_PART_SELECTED);
    lv_obj_set_style_text_font(s_roller, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_roller, lv_color_hex(0x556677), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_roller, &lv_font_montserrat_16, LV_PART_SELECTED);
    lv_obj_set_style_text_color(s_roller, lv_color_white(), LV_PART_SELECTED);

    arrow_btn(parent, LV_SYMBOL_UP,   LV_ALIGN_BOTTOM_LEFT,  30, up_cb);
    arrow_btn(parent, LV_SYMBOL_DOWN, LV_ALIGN_BOTTOM_RIGHT, -30, down_cb);
}

static void menuorder_close(void)
{
    s_roller = NULL;
}

const tool_t tool_menuorder = {
    .name = "Ordenar",
    .icon = LV_SYMBOL_LIST,
    .accent = 0x9B59B6,
    .open = menuorder_open,
    .close = menuorder_close,
};
