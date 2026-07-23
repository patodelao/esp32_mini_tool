/*
 * self_node.h — El minitool publicándose a sí mismo como un nodo más.
 *
 * Era el único equipo del home-lab invisible en su propia vista de Nodos: si
 * se colgaba o perdía el Wi-Fi, nada lo registraba — ni él mismo. Ahora
 * publica lo mismo que exige a los demás:
 *
 *   labo/nodo/minitool/status  -> "online" / "offline"  (retenido, con LWT)
 *   labo/nodo/minitool/ip      -> "192.168.1.107"       (retenido)
 *   labo/sensor/minitool/rssi  -> dBm                   (retenido)
 *   labo/sensor/minitool/heap  -> kB libres             (retenido)
 *   labo/sensor/minitool/uptime-> minutos encendido     (retenido)
 *
 * Como la telemetría sale por los topics de sensores, la tool Sensores le
 * aplica sola sus umbrales: avisa si el Wi-Fi del propio reloj cae por debajo
 * de -85 dBm o si su RAM libre baja de 8 kB.
 *
 * El last-will lo configura mqtt_hub al crear el cliente (tiene que ir en la
 * conexión, no se puede agregar después).
 */
#pragma once

/* Topic del estado. Lo usa mqtt_hub para el last-will. */
#define SELF_NODE_ID           "minitool"
#define SELF_NODE_STATUS_TOPIC "labo/nodo/" SELF_NODE_ID "/status"

#ifdef __cplusplus
extern "C" {
#endif

/* Arranca la publicación periódica. Idempotente. */
void self_node_init(void);

#ifdef __cplusplus
}
#endif
