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








