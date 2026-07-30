#!/usr/bin/env python3
"""
graficar_sensores.py — Lee los CSV del logger y hace UN grafico por sensor.

Cada grafico: titulo legible ("Suelo Pieza"), unidad en el eje Y y el tiempo en
el eje X. Se guardan como PNG (listos para imprimir) en la subcarpeta 'graficos'.

Uso (con el launcher py, que es el Python real de la PC):
  py graficar_sensores.py                # todos los CSV de ~/Documents/homelab-logs
  py graficar_sensores.py archivo.csv    # solo ese archivo
  py graficar_sensores.py C:/ruta/carpeta   # todos los CSV de esa carpeta

Combina varios dias si hay varios CSV: asi ves de minutos a dias en un mismo
grafico. Solo necesita matplotlib (no pandas).
"""
import csv
import glob
import os
import sys
from collections import defaultdict
from datetime import datetime

import matplotlib
matplotlib.use("Agg")          # sin ventana: solo guarda PNG
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

DEFAULT_DIR = os.path.expanduser("~/Documents/homelab-logs")

# --- Nombres y unidades (mismos criterios que el firmware) -------------------
NODOS = {"pieza": "Pieza", "refri": "Refri", "pi": "RetroPi",
         "minitool": "Minitool", "cam": "Cam"}
MAG = {"temp": "Temp", "hum": "Humedad aire", "suelo": "Suelo", "rssi": "WiFi",
       "uptime": "Encendido", "heap": "RAM", "cpu": "CPU", "disco": "Disco",
       "abierta_seg": "Puerta abierta", "suelo_raw": "Crudo"}
UNID = {"temp": "\u00b0C", "hum": "%", "suelo": "%", "rssi": "dBm",
        "uptime": "min", "heap": "kB", "cpu": "%", "disco": "%",
        "abierta_seg": "s"}


def nombre(sensor):                       # "pieza/suelo" -> "Suelo Pieza"
    nodo, _, mag = sensor.partition("/")
    return f"{MAG.get(mag, mag)} {NODOS.get(nodo, nodo)}"


def unidad(sensor):
    return UNID.get(sensor.rpartition("/")[2], "")


def archivos(arg):
    if arg is None:
        return sorted(glob.glob(os.path.join(DEFAULT_DIR, "sensores-*.csv")))
    if os.path.isdir(arg):
        return sorted(glob.glob(os.path.join(arg, "*.csv")))
    return [arg]


def cargar(paths):
    datos = defaultdict(lambda: ([], []))     # sensor -> (tiempos, valores)
    for p in paths:
        with open(p, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                try:
                    t = datetime.fromisoformat(row["fecha_hora"])
                    v = float(row["valor"])
                except (ValueError, KeyError, TypeError):
                    continue                  # fila rara / valor no numerico
                ts, vs = datos[row["sensor"]]
                ts.append(t)
                vs.append(v)
    return datos


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    paths = archivos(arg)
    if not paths:
        print(f"No encontre CSV en {DEFAULT_DIR}. Corre el logger primero.")
        return
    datos = cargar(paths)
    if not datos:
        print("Los CSV no tienen datos numericos.")
        return

    carpeta = os.path.dirname(os.path.abspath(paths[0]))
    out = os.path.join(carpeta, "graficos")
    os.makedirs(out, exist_ok=True)

    print(f"Leidos {len(paths)} CSV, {len(datos)} sensores:\n")
    for sensor in sorted(datos):
        ts, vs = datos[sensor]
        pares = sorted(zip(ts, vs))           # ordenar por tiempo (varios dias)
        ts = [p[0] for p in pares]
        vs = [p[1] for p in pares]

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(ts, vs, linewidth=1.5, color="#2E82C8")
        ax.set_title(nombre(sensor), fontsize=13, fontweight="bold")
        u = unidad(sensor)
        ax.set_ylabel(u if u else "valor")
        ax.set_xlabel("tiempo")
        ax.grid(True, alpha=0.3)
        ax.xaxis.set_major_formatter(mdates.DateFormatter("%d/%m %H:%M"))
        fig.autofmt_xdate()
        fig.tight_layout()

        archivo = "grafico-" + sensor.replace("/", "-") + ".png"
        fig.savefig(os.path.join(out, archivo), dpi=120)
        plt.close(fig)
        print(f"  {nombre(sensor):22s} {len(vs):6d} puntos  -> {archivo}")

    print(f"\nListo. {len(datos)} graficos PNG en:\n  {out}")


if __name__ == "__main__":
    main()
