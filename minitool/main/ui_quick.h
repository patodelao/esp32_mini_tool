/*
 * ui_quick.h — Panel de ajustes rápidos.
 *
 * Se abre deslizando hacia ABAJO desde la carátula, que es el gesto que usa
 * cualquier reloj o teléfono actual. Reúne lo que uno toca a diario y que hoy
 * obliga a entrar a Config y navegar: brillo, silencio y linterna.
 *
 * No duplica la configuración: son atajos a lo mismo que vive en ui_power. Lo
 * único propio es el silencio manual, que apaga las notificaciones "ahora" sin
 * depender del horario del modo noche.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Abre el panel sobre todo lo demás. Se cierra solo al elegir algo que cambia
 * de pantalla, o deslizando hacia arriba. */
void ui_quick_show(void);

/* Cierra el panel si está abierto. */
void ui_quick_hide(void);

bool ui_quick_active(void);

#ifdef __cplusplus
}
#endif
