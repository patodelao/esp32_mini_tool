#!/usr/bin/env python3
"""
pc_presence.py — este PC anuncia si esta encendido, por MQTT (con Last Will).

Por que: detectar el apagado por *ping* es lento y poco fiable. Tras "Apagar",
Windows deja la placa de red respondiendo pings un buen rato (Inicio rapido +
Wake-on-LAN), asi que HA tarda >10 min en enterarse. Con un "Last Will" el
broker avisa el apagado en segundos, y ademas no depende de la IP del PC.

Que hace:
  - al conectar publica  labo/nodo/pc/estado = "online"  (retenido)
  - deja un Will = "offline" (retenido): el broker lo publica SOLO cuando este
    PC se apaga o pierde la red (~20 s en un corte duro, instantaneo si es limpio)
  - publica la config de MQTT Discovery -> HA crea binary_sensor.pc_escritorio solo

Requisitos:  py -m pip install paho-mqtt
Arranque:    atajo en shell:startup ->  pyw "ruta\\pc_presence.py"
             (pyw = sin ventana de consola)
"""
import json
import socket

import paho.mqtt.client as mqtt

BROKER    = "192.168.1.100"    # Mosquitto de la Pi (anonimo en la LAN)
PORT      = 1883
KEEPALIVE = 15                 # corte duro detectado en ~1.5x = ~22 s
T_STATE   = "labo/nodo/pc/estado"
T_DISC    = "homeassistant/binary_sensor/pc_escritorio/config"

# HA agrega el sensor solo con esto (retenido); device_class connectivity:
#   on = online (PC encendido), off = offline (PC apagado)
DISCOVERY = {
    "name": "PC escritorio",
    "unique_id": "pc_escritorio",
    "state_topic": T_STATE,
    "payload_on": "online",
    "payload_off": "offline",
    "device_class": "connectivity",
    "device": {"identifiers": ["pc_escritorio"], "name": "PC escritorio"},
}


def on_connect(client, userdata, flags, reason_code=0, properties=None):
    # firma compatible con paho 1.x (4 args) y 2.x (5 args)
    print("conectado al broker:", reason_code)
    client.publish(T_DISC, json.dumps(DISCOVERY), qos=1, retain=True)
    client.publish(T_STATE, "online", qos=1, retain=True)


def make_client():
    cid = "pc-" + socket.gethostname()
    try:  # paho-mqtt 2.x
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
    except (AttributeError, TypeError):  # paho-mqtt 1.x
        return mqtt.Client(client_id=cid)


def main():
    cli = make_client()
    cli.on_connect = on_connect
    # el "aviso de muerte": queda armado en el broker y se dispara al desconectar
    cli.will_set(T_STATE, "offline", qos=1, retain=True)
    cli.connect(BROKER, PORT, keepalive=KEEPALIVE)
    cli.loop_forever(retry_first_connection=True)   # reconecta solo


if __name__ == "__main__":
    main()
