/*
 * ota_web.h — Actualización del firmware por Wi-Fi.
 *
 * El nodo levanta un servidor HTTP mínimo y recibe el binario por POST. No
 * hace falta un servidor externo ni una nube: se sube desde la misma PC donde
 * compilás, con un curl.
 *
 *     curl -X POST --data-binary @build/esp32cam.bin \
 *          "http://<ip-del-nodo>/update?key=<OTA_PASSWORD>"
 *
 * Este es EL punto del proyecto en su primera etapa: el ESP32-CAM no tiene USB
 * propio y se flashea con un adaptador USB-TTL incómodo. La idea es hacer eso
 * UNA sola vez —el primer flasheo por cable— y de ahí en adelante actualizar
 * siempre por Wi-Fi, igual que el nodo del refri.
 *
 * Por qué así y no como el ESP8266: ArduinoOTA (el mecanismo de "invitación"
 * que usa el nodo de la pieza) es de Arduino, no de ESP-IDF. Acá el modelo se
 * invierte —el que se conecta es tu PC, no el nodo— y no depende de que el
 * firewall de Windows deje entrar conexiones.
 *
 * Seguridad: la clave viaja en la URL sobre HTTP plano, así que sirve para que
 * nadie de la red flashee la cámara por accidente, no contra alguien que esté
 * espiando el tráfico. Para eso haría falta HTTPS con certificados.
 *
 * El firmware se escribe en la ranura que NO está corriendo; el arranque se
 * cambia recién al terminar y verificar. Una subida cortada no rompe nada.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Aviso al empezar/terminar una actualización, para publicarlo al bus. */
typedef void (*ota_web_notify_t)(const char *nivel, const char *msg);

/* Publicar en MQTT desde los controles de la página (el panel de control usa
 * esto para encender/apagar el timelapse escribiendo la config que consume la
 * Pi). La implementa main.c, que es quien tiene el cliente MQTT. */
typedef void (*ota_web_publish_t)(const char *topic, const char *payload, bool retain);

/* Arranca los servidores (idempotente): el de control/OTA en el puerto 80 y uno
 * aparte para el stream MJPEG en el 81 (así el stream no bloquea el OTA ni el
 * panel). 'key' es la clave de la URL de update; NULL/vacía = no se acepta subida. */
esp_err_t ota_web_start(const char *key, ota_web_notify_t notify);

/* Conecta la función de publicación MQTT (llamar tras ota_web_start). */
void ota_web_set_publish(ota_web_publish_t publish);

/* Informa el estado actual del timelapse (lo lee main.c de la config retenida),
 * para mostrarlo en el panel. */
void ota_web_set_timelapse(bool active, int minutes);

#ifdef __cplusplus
}
#endif
