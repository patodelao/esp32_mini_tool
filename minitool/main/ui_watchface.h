/*
 * ui_watchface.h — Carátula de reposo para la pantalla redonda.
 *
 * Se muestra sobre todo lo demás (lv_layer_top) cuando el menú lleva un rato
 * sin actividad: hora grande, fecha, arco de segundos recorriendo el borde e
 * iconos de estado WiFi/BT. Cualquier toque la cierra.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ui_watchface_show(void);
bool ui_watchface_active(void);

/* --- Dato elegible de la carátula -----------------------------------------
 *
 * Abajo a la izquierda hay un hueco que antes mostraba siempre los pasos del
 * día. Los pasos no le sirven a todo el mundo, pero el hueco sí: es el único
 * lugar donde se lee un dato sin tocar la pantalla. Así que ahora se elige qué
 * va ahí — los pasos o cualquier sensor del home-lab.
 *
 * El id vacío ("") significa pasos. La elección se guarda en NVS. */
#define WF_SLOT_STEPS ""

void ui_watchface_set_slot(const char *sensor_id);
void ui_watchface_get_slot(char *out, int out_size);

#ifdef __cplusplus
}
#endif
