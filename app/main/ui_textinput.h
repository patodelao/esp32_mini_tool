/*
 * ui_textinput.h — Prompt modal de texto con teclado en pantalla.
 *
 * Abre un overlay (sobre lv_layer_top) con un título, un textarea y un
 * teclado QWERTY. Al confirmar (✓) llama al callback con el texto y se
 * cierra; al cancelar se cierra sin llamar. Pensado para la pantalla
 * redonda: el teclado ocupa la franja inferior.
 *
 * Debe llamarse desde el contexto de LVGL (callbacks de eventos/timers).
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ui_text_cb_t)(const char *text, void *user_data);

void ui_text_prompt(const char *title, const char *initial, int max_len,
                    bool password, ui_text_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif
