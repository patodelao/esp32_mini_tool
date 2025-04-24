/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

 #include "lvgl.h"

 void create_gradient_circle(void) {
     // Crear un objeto base
     lv_obj_t * circle = lv_obj_create(lv_scr_act());
 
     // Tamaño del círculo
     int size = 240;
     
     lv_obj_set_size(circle, size, size);
     //lv_obj_center(circle);
     lv_obj_align(circle, LV_ALIGN_CENTER, 0, 0);
     // Estilo del fondo: gradiente lineal radial simulado
     lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);  // Hace que se vea como círculo
     lv_obj_set_style_bg_opa(circle, LV_OPA_COVER, 0);
 
     // Gradiente (desde un color central hacia el borde)
     lv_obj_set_style_bg_color(circle, lv_color_hex(0x00CFFF), 0); // Color inicial
     lv_obj_set_style_bg_grad_color(circle, lv_color_hex(0x004E92), 0); // Color final
     lv_obj_set_style_bg_grad_dir(circle, LV_GRAD_DIR_VER, 0);  // Dirección vertical (puedes cambiar a LV_GRAD_DIR_HOR o RADIAL si estuviera disponible)
 
     // Borde opcional
     lv_obj_set_style_border_width(circle, 0, 0); // sin borde
 }
 
 #include "lvgl.h"

void create_gradient_square(void) {
    // Crear un objeto base
    lv_obj_t * square = lv_obj_create(lv_scr_act());

    // Tamaño del cuadrado
    int size = 240;
    
    lv_obj_set_size(square, size, size);
    lv_obj_align(square, LV_ALIGN_CENTER, 0, 0);
    // Estilo del fondo: gradiente lineal
    lv_obj_set_style_radius(square, 0, 0);  // Sin bordes redondeados, forma cuadrada
    lv_obj_set_style_bg_opa(square, LV_OPA_COVER, 0);

    // Gradiente (similar al círculo)
    lv_obj_set_style_bg_color(square, lv_color_hex(0x00CFFF), 0); // Color inicial
    lv_obj_set_style_bg_grad_color(square, lv_color_hex(0x004E92), 0); // Color final
    lv_obj_set_style_bg_grad_dir(square, LV_GRAD_DIR_VER, 0);  // Dirección vertical

    // Borde opcional
    lv_obj_set_style_border_width(square, 0, 0); // Sin borde
}











 #define ROWS 3
 #define COLS 3
 #define ITEM_WIDTH 50
 #define ITEM_HEIGHT 50
 #define ITEM_COUNT (ROWS * COLS)
 
 static lv_obj_t *meter;
 static lv_obj_t *btn;
 static lv_disp_rot_t rotation = LV_DISP_ROT_NONE;
 
 /* Callback para los botones del menú */
 static void menu_btn_cb(lv_event_t *e)
 {
     lv_obj_t *btn = lv_event_get_target(e);
     const char *label = lv_label_get_text(lv_obj_get_child(btn, 0));
     LV_LOG_USER("Botón presionado: %s", label);
 }
 
 /* Callback para hacer scroll circular */
 static void scroll_event_cb(lv_event_t *e)
 {
     lv_obj_t *scroll_obj = lv_event_get_target(e);
     lv_coord_t scroll_y = lv_obj_get_scroll_y(scroll_obj);
 
     // Verificar si llegamos al final o al inicio
     if (scroll_y + lv_obj_get_height(scroll_obj) >= lv_obj_get_height(scroll_obj)) {
         lv_obj_scroll_to(scroll_obj, 0, 0, LV_ANIM_ON);
     } else if (scroll_y <= 0) {
         lv_obj_scroll_to(scroll_obj, 0, lv_obj_get_scroll_bottom(scroll_obj), LV_ANIM_ON);
     }
 }
 
 /* Crear el menú matricial con scroll */
 void example_lvgl_matrix_menu(lv_disp_t *disp)
 {
     lv_obj_t *scr = lv_disp_get_scr_act(disp);
 
     lv_obj_t *cont = lv_obj_create(scr);
     lv_obj_set_size(cont, 320, 240);
     lv_obj_center(cont);
     lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN_WRAP);
     lv_obj_set_scroll_dir(cont, LV_DIR_VER);
     lv_obj_set_style_pad_row(cont, 10, 0);
     lv_obj_set_style_pad_column(cont, 10, 0);
     lv_obj_set_style_pad_all(cont, 10, 0);
     lv_obj_set_style_bg_color(cont, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
 
     for (int i = 0; i < ITEM_COUNT; i++) {
         lv_obj_t *btn = lv_btn_create(cont);
         lv_obj_set_size(btn, ITEM_WIDTH, ITEM_HEIGHT);
         lv_obj_add_event_cb(btn, menu_btn_cb, LV_EVENT_CLICKED, NULL);
 
         lv_obj_t *label = lv_label_create(btn);
         lv_label_set_text_fmt(label, "Item %d", i + 1);
         lv_obj_center(label);
     }
 
     lv_obj_add_event_cb(cont, scroll_event_cb, LV_EVENT_SCROLL, NULL);
 }
 