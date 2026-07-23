/*
 * ota_web.h — Actualización del firmware por Wi-Fi.
 *
 * El nodo levanta un servidor HTTP mínimo y recibe el binario por POST. No
 * hace falta un servidor externo ni una nube: se sube desde la misma PC donde
 * compilás, con un curl.
 *
 *     curl -X POST --data-binary @build/opendoor_alarm.bin \
 *          "http://<ip-del-nodo>/update?key=<OTA_PASSWORD>"
 *
 * Por qué así y no como el ESP8266: ArduinoOTA (el mecanismo de "invitación"
 * que usa el nodo de la pieza) es de Arduino, no de ESP-IDF. Acá el modelo se
 * invierte —el que se conecta es tu PC, no el nodo— y eso tiene una ventaja
 * concreta: no depende de que el firewall de Windows deje entrar conexiones,
 * que fue justo lo que costó hacer andar el otro.
 *
 * Seguridad: la clave viaja en la URL sobre HTTP plano, así que sirve para que
 * nadie de la red flashee el refri por accidente, no contra alguien que esté
 * espiando el tráfico. Para eso haría falta HTTPS con certificados.
 *
 * El firmware se escribe en la ranura que NO está corriendo; el arranque se
 * cambia recién al terminar y verificar. Una subida cortada no rompe nada.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Aviso al empezar/terminar una actualización, para publicarlo al bus. */
typedef void (*ota_web_notify_t)(const char *nivel, const char *msg);

/* Arranca el servidor (idempotente). 'key' es la clave que hay que pasar en
 * la URL; si es NULL o vacía, no se acepta ninguna subida. */
esp_err_t ota_web_start(const char *key, ota_web_notify_t notify);

#ifdef __cplusplus
}
#endif
