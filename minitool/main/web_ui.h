/*
 * web_ui.h — Panel web del home-lab y actualización por Wi-Fi del minitool.
 *
 * Dos cosas que resuelve, ambas de la misma pieza:
 *
 *  1) VER EL HOME-LAB DESDE EL TELÉFONO. La pantalla del reloj mide 240×240 y
 *     hay que tenerlo en la mano. Abriendo http://<ip>/ desde cualquier
 *     navegador de la red se ven todos los sensores con su estado, los nodos y
 *     las últimas alertas, en una pantalla donde entra todo junto.
 *
 *  2) ACTUALIZARLO SIN CABLE. Era el único de los tres equipos que seguía
 *     necesitando USB. Ahora acepta el firmware por POST, igual que el refri:
 *
 *         curl -X POST --data-binary @build/spi_lcd_touch.bin \
 *              "http://<ip>/update?key=<WEB_OTA_KEY>"
 *
 * Requiere la tabla de particiones con dos ranuras (partitions.csv), así que el
 * primer flasheo con esto tiene que ser por cable: cambia el mapa de la flash.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca el servidor (idempotente). Puede llamarse sin Wi-Fi: queda a la
 * espera y atiende en cuanto haya red. */
esp_err_t web_ui_start(void);

#ifdef __cplusplus
}
#endif
