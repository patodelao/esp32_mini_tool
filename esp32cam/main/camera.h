/*
 * camera.h — Inicialización de la cámara del AI-Thinker ESP32-CAM.
 *
 * Separada del servidor HTTP a propósito: acá vive solo el driver (mapa de
 * pines + arranque del sensor), y quien quiera una foto usa esp_camera_fb_get()
 * directamente. El endpoint /foto.jpg está en ota_web.c, sobre el mismo
 * servidor que ya sirve el OTA.
 *
 * camera_init() NO es fatal si falla: si la placa arrancó sin cámara conectada
 * o la PSRAM no montó, el nodo sigue siendo un nodo OTA válido y /foto.jpg
 * responde 503. Un cuelgue de arranque obligaría a flashear por cable, que es
 * justo lo que este proyecto evita.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca el sensor. Idempotente. Devuelve ESP_OK si quedó lista. */
esp_err_t camera_init(void);

/* true si la cámara arrancó bien y se le puede pedir una foto. */
bool camera_ready(void);

#ifdef __cplusplus
}
#endif
