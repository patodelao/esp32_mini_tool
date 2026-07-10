/*
 * ui_menu.h — Menú tipo carrusel que lista las herramientas registradas.
 *
 * ui_menu_show() construye (o vuelve a) el menú. Al tocar una herramienta se
 * abre a pantalla completa; un botón de "volver" regresa al menú.
 * Debe llamarse con el lock de LVGL tomado (bsp_lvgl_lock).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ui_menu_show(void);

#ifdef __cplusplus
}
#endif
