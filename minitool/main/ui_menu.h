/*
 * ui_menu.h — Menú principal con lv_roller para pantalla redonda.
 *
 * create_main_menu() construye (o vuelve a) el menú principal.
 * Debe llamarse con el lock de LVGL tomado (bsp_lvgl_lock).
 */
#pragma once

#include "tool.h"

#ifdef __cplusplus
extern "C" {
#endif

void create_main_menu(void);
void ui_menu_show(void); /* Alias de compatibilidad */

/* Abre una herramienta desde otra (lo usa Config para las que agrupa).
 *
 * El cambio se difiere con lv_async_call: llamarla desde el callback de un
 * botón implicaría borrar los objetos de la tool actual mientras LVGL todavía
 * está procesando su evento.
 *
 * El gesto de volver (deslizar a la derecha) regresa a la tool que la abrió,
 * no al menú principal. */
void ui_menu_open_tool(const tool_t *tool);

#ifdef __cplusplus
}
#endif
