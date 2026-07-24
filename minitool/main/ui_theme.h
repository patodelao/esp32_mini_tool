/*
 * ui_theme.h — Paleta y piezas visuales compartidas.
 *
 * Cada herramienta fue naciendo con sus propios colores escritos a mano, y el
 * resultado es que el mismo significado tiene tonos distintos según la
 * pantalla: dos verdes de "todo bien", dos rojos de alerta, dos grises casi
 * iguales para el texto secundario. Acá se define UNA vez cada rol.
 *
 * Los nombres son semánticos a propósito (UI_OK, UI_ALERT) y no descriptivos
 * (UI_VERDE): si mañana el verde cambia, cambia en un solo lugar y nada queda
 * a medio camino.
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Superficies ---------------------------------------------------------- */
#define UI_SCREEN      0x000814   /* fondo de la pantalla                     */
#define UI_BG          0x0A0E12   /* fondo de overlays a pantalla completa    */
#define UI_CARD        0x1A2733   /* tarjeta / fila de lista                  */
#define UI_CARD_PRESS  0x27384A   /* la misma, pulsada                        */
#define UI_BTN         0x33445A   /* botón redondo                            */
#define UI_BTN_PRESS   0x27384A
#define UI_LINE        0x2A3A48   /* rejillas, bordes, pistas de slider       */

/* --- Texto ---------------------------------------------------------------- */
#define UI_TEXT        0xDDE6F0   /* principal                                */
#define UI_TITLE       0x8FA8C8   /* título de la pantalla                    */
#define UI_MUTED       0x7F8C8D   /* secundario (valores, pies)               */
#define UI_DIM         0x5A6B7A   /* deshabilitado                            */

/* --- Estado --------------------------------------------------------------- */
#define UI_OK          0x35D07F   /* normal / correcto                        */
#define UI_WARN        0xE0A030   /* atención / dato viejo                    */
#define UI_ALERT       0xE74C3C   /* fuera de umbral / crítico                */
#define UI_INFO        0x3498DB
#define UI_ACCENT      0x2E82C8   /* acento por defecto del menú              */

/* --- Piezas --------------------------------------------------------------- */

/* Título de herramienta: mismo tipo, color y altura en todas. Antes cada tool
 * repetía estas cinco líneas con su propia variante. */
lv_obj_t *ui_title(lv_obj_t *parent, const char *text);

/* Fila/tarjeta con esquinas redondeadas y realimentación al tocar. Devuelve el
 * contenedor para colgarle lo que haga falta. */
lv_obj_t *ui_card(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
                  lv_event_cb_t cb, void *user_data);

/* Botón con forma de píldora, el que se usa para acciones al pie. */
lv_obj_t *ui_pill(lv_obj_t *parent, const char *text, uint32_t color,
                  lv_event_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif
