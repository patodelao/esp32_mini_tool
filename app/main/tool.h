/*
 * tool.h — Interfaz común de una "herramienta" del menú.
 *
 * Cada herramienta se implementa en su propio .c exponiendo una instancia
 * `const tool_t` y se registra en tools.c. El menú se encarga de abrirla
 * (open) en un contenedor a pantalla completa y de cerrarla (close) al volver.
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *name;                 /* Nombre visible en el menú */
    const char *icon;                 /* Símbolo LVGL (LV_SYMBOL_*) */
    uint32_t accent;                  /* Color de acento 0xRRGGBB (0 = default) */
    void (*open)(lv_obj_t *parent);   /* Construye la UI dentro de 'parent' */
    void (*close)(void);              /* Libera timers/recursos (puede ser NULL) */
} tool_t;

/* Registro global de herramientas (definido en tools.c). */
extern const tool_t *const g_tools[];
extern const int g_tools_count;

#ifdef __cplusplus
}
#endif
