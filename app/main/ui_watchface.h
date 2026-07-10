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

#ifdef __cplusplus
}
#endif
