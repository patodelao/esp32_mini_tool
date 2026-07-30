/*
 * mongoose_config.h — configuración de Mongoose para este nodo (ESP-IDF).
 *
 * mongoose.h incluye este archivo cuando MG_ARCH no está definido (ver el
 * comentario "keep this include" cerca del inicio de mongoose.h). Acá fijamos el
 * arch ESP32, que hace que Mongoose use la API de sockets sobre lwIP (la misma
 * pila de red que ya usa el resto del firmware) en vez de su stack TCP/IP propio.
 *
 * Solo se usa el broker MQTT; no hace falta TLS, filesystem ni el stack TCP/IP
 * embebido de Mongoose (todos quedan en su default apagado).
 */
#pragma once

#define MG_ARCH        MG_ARCH_ESP32
#define MG_ENABLE_MQTT 1
