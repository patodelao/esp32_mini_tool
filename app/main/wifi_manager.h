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

/* SSID configurado actualmente (copia a buf). */
void wifi_manager_get_ssid(char *buf, size_t len);

/* Guarda credenciales en NVS y las aplica. Si estaba conectado, reconecta.
 * Pasar NULL en cualquiera de los dos deja ese campo como está. */
void wifi_manager_set_credentials(const char *ssid, const char *pass);

#ifdef __cplusplus
}
#endif
