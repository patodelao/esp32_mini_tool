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

/* Apaga la cámara (esp_camera_deinit): detiene el DMA I2S que corre en segundo
 * plano. HAY que llamarlo antes de un OTA: ese DMA accede a PSRAM y choca con
 * las escrituras a flash (que deshabilitan la caché), y el nodo se cae a mitad
 * de la subida. Tras el OTA el nodo reinicia, así que no hace falta re-armarla. */
void camera_stop(void);

#ifdef __cplusplus
}
#endif
