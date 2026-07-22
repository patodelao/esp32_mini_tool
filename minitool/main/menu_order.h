/*
 * menu_order.h — Orden del menú mutable y persistente.
 *
 * El registro g_tools[] es fijo (compilado), pero el ORDEN en que se muestran
 * las herramientas se puede cambiar en tiempo de ejecución y se guarda en NVS,
 * de modo que sobrevive reinicios sin recompilar. El menú y la tool de
 * reordenamiento operan sobre esta capa en vez de sobre g_tools[] directo.
 *
 * "posición" = lugar visible en el menú (0..count-1)
 * "índice"   = índice real dentro de g_tools[]
 */
#pragma once

#include "tool.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Carga el orden desde NVS (o identidad si no hay/queda inválido). Idempotente.
 * Debe llamarse antes de construir el menú. */
void menu_order_load(void);

/* Cantidad de herramientas (== g_tools_count). */
int menu_order_count(void);

/* Herramienta en la posición visible dada. */
const tool_t *menu_order_get(int pos);

/* Índice en g_tools[] de la herramienta en esa posición visible. */
int menu_order_tool_index(int pos);

/* Mueve la herramienta de la posición 'pos' una plaza (dir = -1 arriba,
 * +1 abajo). Persiste el cambio. Devuelve la nueva posición del ítem movido
 * (o 'pos' si no se movió por estar en el borde). */
int menu_order_move(int pos, int dir);

#ifdef __cplusplus
}
#endif
