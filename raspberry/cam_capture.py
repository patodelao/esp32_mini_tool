#!/usr/bin/env python3
"""
cam_capture.py — Agente de captura del nodo cam del home-lab.

Baja http://<cam>/foto.jpg cada X minutos y la guarda con fecha en disco, para
tener un historial navegable. Vive en la Raspberry (que es donde esta el broker
y hay espacio); si la Pi esta apagada, no captura — pero tampoco habria donde
guardar, asi que es coherente.

TODO es configurable desde el minitool via MQTT retenido, con el mismo patron
que los umbrales del suelo:
    labo/config/cam/captura/activo     "1"/"0"     encender/apagar
    labo/config/cam/captura/intervalo  minutos     cada cuanto captura

La IP del cam se toma sola de labo/nodo/cam/ip (retenido); si no la ve, usa el
default. Como feedback, publica labo/sensor/cam/fotos (total guardadas,
retenido), asi el minitool lo muestra en la tool Sensores sin nada extra.
"""
import os
import time
import threading
import urllib.request
import paho.mqtt.client as mqtt

BROKER = "localhost"
FOTOS_DIR = os.path.expanduser("~/homelab/fotos")
MAX_FILES = 3000            # retencion: conserva las mas nuevas y borra el resto
DEFAULT_IP = "192.168.1.104"

state = {"activo": False, "intervalo_min": 10, "cam_ip": DEFAULT_IP}
lock = threading.Lock()


def count_fotos():
    try:
        return len([f for f in os.listdir(FOTOS_DIR) if f.endswith(".jpg")])
    except FileNotFoundError:
        return 0


def prune():
    """Bordea el crecimiento en la SD: deja solo las MAX_FILES mas nuevas.
    Los nombres empiezan con la fecha, asi que ordenar por nombre es ordenar
    por tiempo."""
    try:
        files = sorted(f for f in os.listdir(FOTOS_DIR) if f.endswith(".jpg"))
    except FileNotFoundError:
        return
    while len(files) > MAX_FILES:
        try:
            os.remove(os.path.join(FOTOS_DIR, files.pop(0)))
        except OSError:
            break


def capturar(client):
    with lock:
        ip = state["cam_ip"]
    url = "http://%s/foto.jpg" % ip
    ts = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(FOTOS_DIR, "cam-%s.jpg" % ts)
    try:
        with urllib.request.urlopen(url, timeout=15) as r:
            data = r.read()
        if not data.startswith(b"\xff\xd8"):     # no es un JPEG valido
            print("captura: respuesta no-JPEG, la ignoro")
            return
        with open(path, "wb") as f:
            f.write(data)
        prune()
        n = count_fotos()
        client.publish("labo/sensor/cam/fotos", str(n), qos=1, retain=True)
        print("guardada %s (%d bytes), total %d" % (path, len(data), n))
    except Exception as e:
        print("captura fallo (cam apagado o sin red?): %s" % e)


def on_connect(client, userdata, flags, rc):
    client.subscribe("labo/config/cam/captura/#")
    client.subscribe("labo/nodo/cam/ip")
    client.publish("labo/sensor/cam/fotos", str(count_fotos()), qos=1, retain=True)
    print("conectado al broker; suscrito a la config de captura")


def on_message(client, userdata, msg):
    p = msg.payload.decode(errors="ignore").strip()
    with lock:
        if msg.topic.endswith("/activo"):
            state["activo"] = p in ("1", "true", "on", "ON")
        elif msg.topic.endswith("/intervalo"):
            try:
                state["intervalo_min"] = max(1, int(float(p)))
            except ValueError:
                pass
        elif msg.topic == "labo/nodo/cam/ip" and p:
            state["cam_ip"] = p
    print("config -> %s" % state)


def loop_capturas(client):
    last = 0.0
    while True:
        time.sleep(2)
        with lock:
            activo = state["activo"]
            iv = state["intervalo_min"] * 60
        if activo and time.time() - last >= iv:
            last = time.time()
            capturar(client)


def main():
    os.makedirs(FOTOS_DIR, exist_ok=True)
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    threading.Thread(target=loop_capturas, args=(client,), daemon=True).start()
    while True:
        try:
            client.connect(BROKER, 1883, keepalive=30)
            client.loop_forever()
        except Exception as e:
            print("broker no disponible (%s); reintento en 10 s" % e)
            time.sleep(10)


if __name__ == "__main__":
    main()
