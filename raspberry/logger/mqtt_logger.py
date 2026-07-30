#!/usr/bin/env python3
"""
mqtt_logger.py — Registra todas las lecturas de sensores del home-lab a CSV.

Se suscribe a labo/sensor/# en el broker (el refri/opendoor, 192.168.1.108) y va
appendeando cada lectura a un CSV por día. Cada fila: fecha/hora, topic, valor.
Formato "largo" (una fila por lectura): es trivial de escribir y en Excel se
filtra/pivota por sensor y por rango de fechas (de minutos a días) sin drama.

Salida:  ~/homelab/logs/sensores-AAAA-MM-DD.csv   (uno por día)

Solo necesita paho-mqtt (ya instalado en la Pi: python3-paho-mqtt). Puede correr
en la Pi como servicio systemd (ver homelab-logger.service) o suelto en la PC
con:  python mqtt_logger.py
"""
import csv
import os
import time
from datetime import datetime

import paho.mqtt.client as mqtt

BROKER   = os.environ.get("BROKER", "192.168.1.108")   # el refri = broker
PORT     = int(os.environ.get("PORT", "1883"))
TOPIC    = "labo/sensor/#"
LOG_DIR  = os.path.expanduser(os.environ.get("LOG_DIR", "~/homelab/logs"))

os.makedirs(LOG_DIR, exist_ok=True)


def csv_path():
    """Ruta del CSV del día de hoy (rota solo al cambiar de día)."""
    return os.path.join(LOG_DIR, f"sensores-{datetime.now():%Y-%m-%d}.csv")


def append_row(topic, value):
    path = csv_path()
    nuevo = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(["fecha_hora", "topic", "sensor", "valor"])  # cabecera
        sensor = topic[len("labo/sensor/"):]      # "pieza/suelo"
        w.writerow([datetime.now().isoformat(timespec="seconds"), topic, sensor, value])


def on_connect(client, userdata, flags, rc):
    client.subscribe(TOPIC, qos=0)
    print(f"[logger] conectado a {BROKER}:{PORT}, suscrito a {TOPIC}")


def on_message(client, userdata, msg):
    val = msg.payload.decode("utf-8", "replace").strip()
    if val == "":
        return                       # payload vacío = borrado de retenido, ignorar
    append_row(msg.topic, val)


def main():
    client = mqtt.Client(client_id="homelab-logger")
    client.on_connect = on_connect
    client.on_message = on_message
    # Reintenta solo si el broker no está (opendoor apagado/OTA).
    while True:
        try:
            client.connect(BROKER, PORT, keepalive=60)
            client.loop_forever()
        except Exception as e:
            print(f"[logger] sin broker ({e}), reintento en 10 s")
            time.sleep(10)


if __name__ == "__main__":
    main()
