#!/usr/bin/env python3
"""MQTT Discovery para HA desde los topics labo/: sensores, estado de nodos y
puerta. Agrupa por nodo como dispositivo. Retenido."""
import json, time
import paho.mqtt.client as mqtt

BROKER, PORT, COLLECT_S = "127.0.0.1", 1883, 70

UNIT = {"temp": "\u00b0C", "hum": "%", "suelo": "%", "rssi": "dBm", "cpu": "%",
        "disco": "%", "uptime": "min", "heap": "kB", "abierta_seg": "s"}
DEVCLASS = {"temp": "temperature", "hum": "humidity", "suelo": "moisture",
            "rssi": "signal_strength"}
PRETTY = {"temp": "Temperatura", "hum": "Humedad", "suelo": "Suelo", "rssi": "Senal",
          "cpu": "CPU", "disco": "Disco", "uptime": "Encendido", "heap": "Heap libre",
          "fotos": "Fotos", "mqtt_clientes": "Clientes MQTT",
          "mqtt_zombies": "Zombies MQTT", "abierta_seg": "Abierta"}
NODE = {"pieza": "Pieza", "refri": "Refri", "cam": "Cam", "pi": "RetroPi",
        "minitool": "Minitool"}
KNOWN_DOORS = {"refri"}

sensors, nodes, doors = set(), set(), set()

def on_message(c, u, m):
    t = m.topic
    if t.startswith("labo/sensor/"):
        sid = t[len("labo/sensor/"):]
        try:
            float(m.payload.decode().strip()); sensors.add(sid)
        except Exception:
            pass
    elif t.startswith("labo/nodo/") and t.endswith("/status"):
        nodes.add(t[len("labo/nodo/"):-len("/status")])
    elif t.startswith("labo/nodo/") and t.endswith("/puerta"):
        doors.add(t[len("labo/nodo/"):-len("/puerta")])

try:
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
except AttributeError:
    cli = mqtt.Client()
cli.on_message = on_message
cli.connect(BROKER, PORT, 30)
for f in ("labo/sensor/#", "labo/nodo/+/status", "labo/nodo/+/puerta"):
    cli.subscribe(f)
cli.loop_start()
print(f"Escuchando {COLLECT_S}s...")
time.sleep(COLLECT_S)

def dev(node):
    return {"identifiers": [f"labo_{node}"],
            "name": NODE.get(node, node.capitalize()), "manufacturer": "home-lab"}

n = 0
for sid in sorted(sensors):
    node, leaf = sid.split("/")[0], sid.split("/")[-1]
    obj = "labo_" + sid.replace("/", "_")
    cfg = {"name": PRETTY.get(leaf, leaf.capitalize()),
           "state_topic": f"labo/sensor/{sid}", "unique_id": obj,
           "state_class": "measurement", "device": dev(node)}
    if UNIT.get(leaf): cfg["unit_of_measurement"] = UNIT[leaf]
    if DEVCLASS.get(leaf): cfg["device_class"] = DEVCLASS[leaf]
    cli.publish(f"homeassistant/sensor/{obj}/config", json.dumps(cfg), 1, True); n += 1

for node in sorted(nodes):
    obj = f"labo_{node}_status"
    cfg = {"name": "Estado", "state_topic": f"labo/nodo/{node}/status",
           "payload_on": "online", "payload_off": "offline",
           "device_class": "connectivity", "unique_id": obj, "device": dev(node)}
    cli.publish(f"homeassistant/binary_sensor/{obj}/config", json.dumps(cfg), 1, True); n += 1

for node in sorted(doors | (KNOWN_DOORS & nodes)):
    obj = f"labo_{node}_puerta"
    cfg = {"name": "Puerta", "state_topic": f"labo/nodo/{node}/puerta",
           "payload_on": "ABIERTO", "payload_off": "CERRADO", "device_class": "door",
           "unique_id": obj, "device": dev(node)}
    cli.publish(f"homeassistant/binary_sensor/{obj}/config", json.dumps(cfg), 1, True); n += 1

time.sleep(2)
cli.loop_stop(); cli.disconnect()
print(f"Listo: {n} entidades ({len(sensors)} sensores, {len(nodes)} nodos, "
      f"{len(doors | (KNOWN_DOORS & nodes))} puertas).")
