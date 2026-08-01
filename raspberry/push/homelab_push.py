#!/usr/bin/env python3
"""
homelab_push.py — Reenvía las alertas del bus MQTT a tu teléfono como push.

Se suscribe a labo/alerta/# en el broker (el refri/opendoor, 192.168.1.108) y por
cada alerta {origen, nivel, msg} hace un POST a ntfy.sh. Te llega como
notificación al celu estés donde estés, con la app ntfy instalada y suscripta al
mismo topic — así "monitorear desde afuera" incluye enterarte sin mirar nada.

Es lo mismo que ya muestra el minitool como toast flotante, pero puenteado a la
nube para que salga de la LAN. Como el bus es multicanal (labo/alerta/<lo que
sea>), sirve para el refri, la pieza y cualquier equipo que se sume, sin tocar
este script.

Config por variables de entorno (ver homelab-push.service / push.env.example):
  BROKER, PORT           broker MQTT (default el refri, 192.168.1.108:1883)
  MQTT_USER, MQTT_PASS   credenciales del broker (auth activa en la flota)
  NTFY_URL               p.ej. https://ntfy.sh/mi-topic-largo-y-secreto-42
                         El topic ES tu "clave": cualquiera que lo sepa ve tus
                         alertas. Elegí uno largo y difícil de adivinar.
  MIN_LEVEL              nivel mínimo a empujar: alarma | aviso | ok (default aviso)

Dependencias:  pip3 install paho-mqtt requests
"""
import json
import os
import time

import paho.mqtt.client as mqtt
import requests

BROKER    = os.environ.get("BROKER", "192.168.1.108")
PORT      = int(os.environ.get("PORT", "1883"))
MQTT_USER = os.environ.get("MQTT_USER", "")
MQTT_PASS = os.environ.get("MQTT_PASS", "")
NTFY_URL  = os.environ.get("NTFY_URL", "")
MIN_LEVEL = os.environ.get("MIN_LEVEL", "aviso")
TOPIC     = "labo/alerta/#"

# nivel del bus -> (prioridad ntfy, tag/emoji, orden). El orden filtra por MIN_LEVEL.
NIVEL = {
    "ok":     ("default", "white_check_mark", 0),
    "aviso":  ("high",    "warning",          1),
    "alarma": ("urgent",  "rotating_light",   2),
}
MIN_ORD = NIVEL.get(MIN_LEVEL, NIVEL["aviso"])[2]


def on_connect(client, userdata, flags, rc):
    client.subscribe(TOPIC, qos=0)
    print(f"[push] conectado a {BROKER}:{PORT}, suscrito a {TOPIC} (rc={rc})")


def on_message(client, userdata, msg):
    if not NTFY_URL:
        print("[push] falta NTFY_URL: no hay a dónde empujar")
        return
    try:
        d = json.loads(msg.payload.decode("utf-8", "replace"))
    except (ValueError, UnicodeError):
        return  # payload que no es JSON del bus, ignorar

    origen = str(d.get("origen", "Home-lab"))
    nivel  = str(d.get("nivel", "aviso"))
    texto  = str(d.get("msg", ""))

    prio, tag, ordn = NIVEL.get(nivel, ("high", "bell", 1))
    if ordn < MIN_ORD:
        return  # por debajo del umbral configurado, no molestar

    try:
        requests.post(
            NTFY_URL,
            data=texto.encode("utf-8"),
            headers={"Title": origen, "Priority": prio, "Tags": tag},
            timeout=5,
        )
        print(f"[push] -> ntfy: [{nivel}] {origen}: {texto}")
    except requests.RequestException as e:
        print(f"[push] error enviando a ntfy: {e}")


def main():
    client = mqtt.Client(client_id="homelab-push")
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message

    while True:  # reconexión propia: si el broker no está, reintentar
        try:
            client.connect(BROKER, PORT, keepalive=60)
            client.loop_forever()
        except (OSError, TimeoutError) as e:
            print(f"[push] sin broker ({e}), reintento en 10 s")
            time.sleep(10)


if __name__ == "__main__":
    main()
