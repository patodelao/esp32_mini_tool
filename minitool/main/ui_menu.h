/*
 * ui_menu.h — Menú principal con lv_roller para pantalla redonda.
 *
 * create_main_menu() construye (o vuelve a) el menú principal.
 * Debe llamarse con el lock de LVGL tomado (bsp_lvgl_lock).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void create_main_menu(void);
void ui_menu_show(void); /* Alias de compatibilidad */

#ifdef __cplusplus
}
#endif
