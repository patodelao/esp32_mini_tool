/*
 * wifi_manager.h — Gestión de Wi-Fi en modo estación + hora por SNTP.
 *
 * La conexión es NO bloqueante: wifi_manager_connect() lanza el intento y
 * retorna de inmediato; el estado se consulta con wifi_manager_is_connected().
 * Las credenciales se guardan en NVS (namespace "cfg") y sobreviven reinicios.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_SSID_MAX 32
#define WIFI_PASS_MAX 64

/* Inicializa NVS, la pila de red y el Wi-Fi (sin conectar todavía). */
void wifi_manager_init(void);

/* Lanza (no bloquea) el intento de conexión y arranca SNTP. */
void wifi_manager_connect(void);

/* Desconecta del AP. */
void wifi_manager_disconnect(void);

/* true si hay IP asignada. */
bool wifi_manager_is_connected(void);

/* true si el reloj ESTÁ INTENTANDO conectarse (aunque todavía no lo logre).
 * Sin esto, "no conectado" tapa dos situaciones muy distintas: que la clave
 * esté mal o el router lejos, o que directamente nadie le pidió conectarse. La
 * pantalla de Estado las distingue. */
bool wifi_manager_should_connect(void);

/* SSID configurado actualmente (copia a buf). */
void wifi_manager_get_ssid(char *buf, size_t len);

/* Contraseña configurada actualmente (copia a buf). Usada por la tool QR WiFi
 * para generar el código de unión a la red. */
void wifi_manager_get_pass(char *buf, size_t len);

/* Guarda credenciales en NVS y las aplica. Si estaba conectado, reconecta.
 * Pasar NULL en cualquiera de los dos deja ese campo como está. */
void wifi_manager_set_credentials(const char *ssid, const char *pass);

#ifdef __cplusplus
}
#endif
