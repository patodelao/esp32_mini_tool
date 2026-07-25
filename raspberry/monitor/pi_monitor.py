#!/usr/bin/env python3
"""
pi_monitor.py — La Pi se auto-monitorea publicando a MQTT como un nodo más.

Sigue las convenciones de la flota (las mismas que usan el minitool y los ESP),
así que la Pi aparece sola en las tools **Nodos** y **Sensores** del minitool,
sin nada de config extra allá:

    labo/nodo/pi/status     online / offline   [retain + LWT]
    labo/nodo/pi/ip         IP en la LAN       [retain]
    labo/sensor/pi/temp     temperatura CPU, °C [retain]
    labo/sensor/pi/uptime   encendido, min      [retain]
    labo/sensor/pi/rssi     señal WiFi, dBm      [retain]
    labo/sensor/pi/cpu      uso de CPU, %        [retain]
    labo/sensor/pi/disco    uso de / , %         [retain]

Solo biblioteca estándar + paho-mqtt (ya instalado para cam_capture.py).
Corre como servicio systemd (homelab-monitor).
"""
import os
import time
import socket
import subprocess
import paho.mqtt.client as mqtt

BROKER = "localhost"
NODE = "pi"
PERIODO_S = 30

T_STATUS = "labo/nodo/%s/status" % NODE
T_IP = "labo/nodo/%s/ip" % NODE
T_SENSOR = "labo/sensor/%s/" % NODE


def cpu_temp_c():
    """Temperatura de la CPU en °C. /sys es más barato que vcgencmd."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def uptime_min():
    try:
        with open("/proc/uptime") as f:
            return int(float(f.read().split()[0]) / 60)
    except (OSError, ValueError):
        return None


def wifi_rssi_dbm():
    """Nivel de señal de wlan0 en dBm, desde /proc/net/wireless."""
    try:
        with open("/proc/net/wireless") as f:
            for line in f:
                if line.strip().startswith("wlan0"):
                    # cols: iface status link level noise ... ; level en la 4ta
                    lvl = line.split()[3]
                    return int(float(lvl.rstrip(".")))
    except (OSError, ValueError, IndexError):
        pass
    return None


def _cpu_totals():
    with open("/proc/stat") as f:
        parts = f.readline().split()[1:]      # user nice system idle iowait ...
    vals = [int(x) for x in parts]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
    return sum(vals), idle


def cpu_pct():
    """Uso de CPU en % sobre una ventana corta."""
    try:
        t0, i0 = _cpu_totals()
        time.sleep(0.5)
        t1, i1 = _cpu_totals()
        dt = t1 - t0
        if dt <= 0:
            return None
        return round(100.0 * (1.0 - (i1 - i0) / dt), 1)
    except (OSError, ValueError, IndexError):
        return None


def disco_pct():
    try:
        st = os.statvfs("/")
        usado = (st.f_blocks - st.f_bfree)
        total = st.f_blocks
        return round(100.0 * usado / total) if total else None
    except OSError:
        return None


def lan_ip():
    """IP de salida a la LAN (sin resolver DNS)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 80))
        return s.getsockname()[0]
    except OSError:
        return "0.0.0.0"
    finally:
        s.close()


def pub(client, topic, value):
    if value is not None:
        client.publish(topic, str(value), qos=1, retain=True)


def on_connect(client, userdata, flags, rc):
    client.publish(T_STATUS, "online", qos=1, retain=True)
    pub(client, T_IP, lan_ip())
    print("conectado; publicando como nodo '%s'" % NODE)


def main():
    client = mqtt.Client()
    # Last-Will: si el agente/la Pi cae sin avisar, el broker publica "offline".
    client.will_set(T_STATUS, "offline", qos=1, retain=True)
    client.on_connect = on_connect
    while True:
        try:
            client.connect(BROKER, 1883, keepalive=60)
            break
        except OSError as e:
            print("broker no disponible (%s); reintento en 10 s" % e)
            time.sleep(10)
    client.loop_start()
    try:
        while True:
            # El "online" va en CADA vuelta, no solo al conectar: si el last-will
            # marcó offline por un parpadeo de red, esto lo revierte enseguida.
            client.publish(T_STATUS, "online", qos=1, retain=True)
            pub(client, T_SENSOR + "temp", cpu_temp_c())
            pub(client, T_SENSOR + "uptime", uptime_min())
            pub(client, T_SENSOR + "rssi", wifi_rssi_dbm())
            pub(client, T_SENSOR + "cpu", cpu_pct())      # cpu_pct duerme 0.5 s
            pub(client, T_SENSOR + "disco", disco_pct())
            pub(client, T_IP, lan_ip())
            time.sleep(PERIODO_S)
    finally:
        client.loop_stop()


if __name__ == "__main__":
    main()
