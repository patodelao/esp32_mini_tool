#!/usr/bin/env python3
"""
homelab_panel.py — Panel táctil para la TFT MHS35 (480x320, /dev/fb0).

Launcher con dos modos, dibujado directo al framebuffer con Pillow (sin
navegador, fluido en la SPI):

  - MENU: dos botones grandes -> Home-lab | RetroPie (se elige con el TOUCH).
  - PANEL (Home-lab): tablero en vivo del home-lab leyendo el MQTT local.
  - RetroPie: lanza emulationstation (mando); si no está, avisa.

El touch (ADS7846) solo maneja el menú/panel. Tema oscuro, letras grandes.
"""
import os
import time
import json
import mmap
import statistics
import threading
import subprocess
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import paho.mqtt.client as mqtt
import evdev
from evdev import ecodes

# --------------------------------------------------------------------------- #
FB_PATH = "/dev/fb0"
BROKER, PORT = "127.0.0.1", 1883

# Calibración del touch resistivo (ADS7846) para la MHS35 rotada 270°.
# El panel imprime  TAP raw(...)-> screen(...)  en consola: tocá las 4 esquinas
# y ajustá estos tres si los toques caen corridos/espejados.
TS_SWAP = True
TS_INVX = False
TS_INVY = True

# Paleta (tema oscuro, igual que el minitool)
BG     = (0, 8, 20)
CARD   = (24, 37, 50)
TEXT   = (233, 240, 248)
TITLE  = (143, 168, 200)
MUTED  = (120, 134, 148)
OK     = (53, 208, 127)
ALERT  = (231, 76, 60)
WARN   = (224, 160, 48)
INFO   = (74, 168, 255)
TEMPC  = (240, 137, 74)
SOIL   = (96, 200, 160)
VIOLET = (155, 89, 182)


def load_pointercal(path="/etc/pointercal"):
    """Calibración de tslib: 7 enteros (transformación afín raw->pantalla), que
    genera  ts_calibrate . Si existe, se usa; si no, cae al mapeo por flags."""
    try:
        v = [int(x) for x in open(path).read().split()[:7]]
        if len(v) == 7 and v[6] != 0:
            return v
    except Exception:
        pass
    return None


def _font(sz, bold=False):
    p = "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else "")
    try:
        return ImageFont.truetype(p, sz)
    except Exception:
        return ImageFont.load_default()


F_MENU  = _font(32, True)
F_VAL   = _font(44, True)   # valores del ambiente: BIEN grandes
F_BIG   = _font(32, True)   # puerta / nodos
F_TITLE = _font(22, True)
F_LBL   = _font(17, True)
F_SM    = _font(15)
F_BACK  = _font(20, True)


# --------------------------------------------------------------------------- #
class FrameBuffer:
    def __init__(self, path=FB_PATH):
        try:
            vs = open("/sys/class/graphics/fb0/virtual_size").read().strip().split(",")
            self.w, self.h = int(vs[0]), int(vs[1])
            self.stride = int(open("/sys/class/graphics/fb0/stride").read().strip())
        except Exception:
            self.w, self.h, self.stride = 480, 320, 480 * 2
        self.fd = os.open(path, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, self.stride * self.h)

    def show(self, img):
        a = np.asarray(img, dtype=np.uint16)
        rgb565 = ((a[:, :, 0] >> 3) << 11) | ((a[:, :, 1] >> 2) << 5) | (a[:, :, 2] >> 3)
        data = rgb565.astype("<u2").tobytes()
        line = self.w * 2
        if self.stride == line:
            self.mm.seek(0)
            self.mm.write(data)
        else:
            for y in range(self.h):
                self.mm.seek(y * self.stride)
                self.mm.write(data[y * line:(y + 1) * line])


class Touch:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.dev = None
        for p in evdev.list_devices():
            d = evdev.InputDevice(p)
            if "ADS7846" in d.name or "Touchscreen" in d.name:
                self.dev = d
                break
        self.minx, self.maxx, self.miny, self.maxy = 0, 4095, 0, 4095
        if self.dev:
            caps = dict(self.dev.capabilities().get(ecodes.EV_ABS, []))
            ax, ay = caps.get(ecodes.ABS_X), caps.get(ecodes.ABS_Y)
            if ax:
                self.minx, self.maxx = ax.min, ax.max
            if ay:
                self.miny, self.maxy = ay.min, ay.max
        self._tap = None
        self._lock = threading.Lock()
        self.last_raw = (0, 0)      # para el overlay de calibración
        self.last_scr = (-1, -1)
        self.cal = load_pointercal()   # afín de tslib (ts_calibrate), o None

    def _map(self, rx, ry):
        if self.cal:                   # calibración tslib: afín completa
            a0, a1, a2, a3, a4, a5, a6 = self.cal
            sx = (a2 + a0 * rx + a1 * ry) / a6
            sy = (a5 + a3 * rx + a4 * ry) / a6
        else:                          # fallback: normalizar + swap/invert
            nx = (rx - self.minx) / max(1, (self.maxx - self.minx))
            ny = (ry - self.miny) / max(1, (self.maxy - self.miny))
            if TS_SWAP:
                nx, ny = ny, nx
            if TS_INVX:
                nx = 1.0 - nx
            if TS_INVY:
                ny = 1.0 - ny
            sx, sy = nx * self.w, ny * self.h
        return (min(self.w - 1, max(0, int(sx))),
                min(self.h - 1, max(0, int(sy))))

    # Filtrado de ruido (reemplaza el debounce/settle que el overlay mhs35 no
    # trae): por cada toque juntamos varias muestras, descartamos las primeras
    # (no asentadas) y usamos la MEDIANA -> mata outliers y jitter.
    SETTLE = 2      # lecturas iniciales a descartar
    NEED = 5        # muestras (tras el settle) para la mediana

    def run(self):
        if not self.dev:
            return
        press_btns = (ecodes.BTN_TOUCH, ecodes.BTN_LEFT)
        rx = ry = None
        buf = []
        down = False
        fired = False
        last_t = 0.0

        def emit():
            nonlocal fired, last_t
            if fired or not buf:
                return
            fired = True
            pts = buf[self.SETTLE:] if len(buf) > self.SETTLE else buf
            mx = int(statistics.median(p[0] for p in pts))
            my = int(statistics.median(p[1] for p in pts))
            now = time.time()
            if now - last_t <= 0.15:            # antirrebote entre toques
                return
            last_t = now
            sx, sy = self._map(mx, my)
            self.last_raw = (mx, my)
            self.last_scr = (sx, sy)
            print("TAP raw(%d,%d) n=%d -> screen(%d,%d)" % (mx, my, len(pts), sx, sy),
                  flush=True)
            with self._lock:
                self._tap = (sx, sy)

        for ev in self.dev.read_loop():
            if ev.type == ecodes.EV_ABS:
                if ev.code == ecodes.ABS_X:
                    rx = ev.value
                elif ev.code == ecodes.ABS_Y:
                    ry = ev.value
            elif ev.type == ecodes.EV_SYN and ev.code == ecodes.SYN_REPORT:
                # Fin de frame: si hay dedo y coords, guardar la muestra.
                if down and not fired and rx is not None and ry is not None:
                    buf.append((rx, ry))
                    if len(buf) >= self.SETTLE + self.NEED:
                        emit()                  # ya juntamos suficientes
            elif ev.type == ecodes.EV_KEY and ev.code in press_btns:
                if ev.value == 1:               # apoya el dedo
                    down, fired, buf = True, False, []
                elif ev.value == 0:             # levanta
                    if not fired:               # tap corto: usar lo que haya
                        emit()
                    down = False

    def get_tap(self):
        with self._lock:
            t, self._tap = self._tap, None
            return t


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.temp = self.hum = self.suelo = None
        self.puerta = None
        self.nodes = {}
        self.alert = None
        self.dirty = True       # hay algo nuevo que redibujar

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)
            self.dirty = True


def start_mqtt(st):
    def on_msg(c, u, m):
        t = m.topic
        try:
            p = m.payload.decode().strip()
        except Exception:
            return
        if t == "labo/sensor/pieza/temp":
            st._set(temp=p)
        elif t == "labo/sensor/pieza/hum":
            st._set(hum=p)
        elif t == "labo/sensor/pieza/suelo":
            st._set(suelo=p)
        elif t == "labo/nodo/refri/puerta":
            st._set(puerta=p)
        elif t.startswith("labo/nodo/") and t.endswith("/status"):
            with st.lock:
                st.nodes[t[len("labo/nodo/"):-len("/status")]] = p
                st.dirty = True
        elif t.startswith("labo/alerta/"):
            try:
                j = json.loads(p)
                st._set(alert=(j.get("origen", "?"), j.get("nivel", "info"), j.get("msg", "")))
            except Exception:
                pass

    try:
        cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    except AttributeError:
        cli = mqtt.Client()
    cli.on_message = on_msg
    cli.connect(BROKER, PORT, 30)
    for f in ("labo/sensor/pieza/#", "labo/nodo/+/status", "labo/nodo/refri/puerta", "labo/alerta/#"):
        cli.subscribe(f)
    cli.loop_start()
    return cli


# --------------------------------------------------------------------------- #
# Dibujo
# --------------------------------------------------------------------------- #
def card(d, x, y, w, h, fill=CARD, radius=14, outline=None, ow=2):
    d.rounded_rectangle([x, y, x + w, y + h], radius=radius, fill=fill,
                        outline=outline, width=(ow if outline else 0))


def tc(d, cx, cy, s, font, fill):
    b = d.textbbox((0, 0), s, font=font)
    d.text((cx - (b[2] - b[0]) / 2, cy - (b[3] - b[1]) / 2 - b[1]), s, font=font, fill=fill)


def dot(d, cx, cy, r, fill):
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill)


def icon_home(d, cx, cy, s, acc):
    d.polygon([(cx - s, cy - s * 0.02), (cx, cy - s * 0.85), (cx + s, cy - s * 0.02)], fill=(255, 255, 255))
    d.rectangle([cx - s * 0.66, cy - s * 0.02, cx + s * 0.66, cy + s * 0.66], fill=(255, 255, 255))
    d.rectangle([cx - s * 0.17, cy + s * 0.18, cx + s * 0.17, cy + s * 0.66], fill=acc)


def icon_pad(d, cx, cy, s, acc):
    d.rounded_rectangle([cx - s, cy - s * 0.42, cx + s, cy + s * 0.42], radius=int(s * 0.42), fill=(255, 255, 255))
    d.rectangle([cx - s * 0.62, cy - s * 0.09, cx - s * 0.20, cy + s * 0.09], fill=acc)
    d.rectangle([cx - s * 0.48, cy - s * 0.24, cx - s * 0.34, cy + s * 0.24], fill=acc)
    dot(d, cx + s * 0.34, cy, int(s * 0.11), acc)
    dot(d, cx + s * 0.60, cy, int(s * 0.11), acc)


def draw_menu(st):
    img = Image.new("RGB", (480, 320), BG)
    d = ImageDraw.Draw(img)
    tc(d, 240, 30, "HOME-LAB", F_MENU, TITLE)

    # Home-lab (verde)
    card(d, 26, 70, 200, 216, radius=20, outline=OK, ow=3)
    dot(d, 126, 148, 44, OK)
    icon_home(d, 126, 150, 30, OK)
    tc(d, 126, 244, "Home-lab", F_MENU, TEXT)

    # RetroPie (violeta o gris si no está)
    has_es = shutil.which("emulationstation") is not None
    acc = VIOLET if has_es else MUTED
    card(d, 254, 70, 200, 216, radius=20, outline=acc, ow=3)
    dot(d, 354, 148, 44, acc)
    icon_pad(d, 354, 150, 30, acc)
    tc(d, 354, 244, "RetroPie", F_MENU, TEXT if has_es else MUTED)
    if not has_es:
        tc(d, 354, 272, "proximamente", F_SM, MUTED)

    return img, [((26, 70, 226, 286), "homelab"), ((254, 70, 454, 286), "retropie")]


def _v(v, suf):
    return ("%s%s" % (v, suf)) if v is not None else "--"


def draw_panel(st):
    with st.lock:
        temp, hum, suelo = st.temp, st.hum, st.suelo
        puerta, nodes, alert = st.puerta, dict(st.nodes), st.alert

    img = Image.new("RGB", (480, 320), BG)
    d = ImageDraw.Draw(img)

    # Header + botón VOLVER grande (zona táctil generosa)
    card(d, 8, 7, 150, 40, radius=20, fill=CARD, outline=TITLE, ow=2)
    tc(d, 83, 27, "‹ Volver", F_BACK, TITLE)
    tc(d, 300, 27, "HOME-LAB", F_TITLE, TITLE)
    d.text((410, 15), time.strftime("%H:%M"), font=F_TITLE, fill=TEXT)
    back = [((0, 0, 185, 54), "menu")]

    # Ambiente: 3 tarjetas grandes con borde de acento
    tiles = [("TEMP", _v(temp, "°"), TEMPC),
             ("HUMEDAD", _v(hum, "%"), INFO),
             ("SUELO", _v(suelo, "%"), SOIL)]
    x = 8
    for lbl, val, acc in tiles:
        card(d, x, 58, 150, 108, radius=14, outline=acc, ow=2)
        tc(d, x + 75, 78, lbl, F_LBL, acc)
        tc(d, x + 75, 122, val, F_VAL, TEXT)
        x += 157

    # Puerta refri + Nodos
    door_open = (puerta == "ABIERTO")
    dcol = ALERT if door_open else (OK if puerta == "CERRADO" else MUTED)
    card(d, 8, 174, 226, 76)
    tc(d, 121, 192, "PUERTA REFRI", F_LBL, MUTED)
    dot(d, 46, 224, 10, dcol)
    tc(d, 130, 224, puerta if puerta else "sin datos", F_BIG, dcol)

    online = sum(1 for v in nodes.values() if v == "online")
    total = len(nodes)
    ncol = OK if (total and online == total) else (ALERT if online == 0 else WARN)
    card(d, 246, 174, 226, 76)
    tc(d, 359, 192, "NODOS ONLINE", F_LBL, MUTED)
    dot(d, 300, 224, 10, ncol)
    tc(d, 372, 224, "%d / %d" % (online, total), F_BIG, ncol)

    # Última alerta
    card(d, 8, 258, 464, 54)
    if alert:
        origen, nivel, msg = alert
        acol = {"alarma": ALERT, "aviso": WARN, "ok": OK}.get(nivel, INFO)
        dot(d, 28, 285, 8, acol)
        d.text((46, 274), ("%s: %s" % (origen, msg))[:50], font=F_LBL, fill=acol)
    else:
        d.text((20, 274), "sin alertas", font=F_LBL, fill=MUTED)

    return img, back


def inside(rect, pt):
    return rect[0] <= pt[0] <= rect[2] and rect[1] <= pt[1] <= rect[3]


def draw_tap_overlay(img, raw, scr):
    """Ayuda de calibración: dibuja un crosshair donde cayó el último toque y
    una lectura de coordenadas (raw + pantalla) abajo. Así se calibra mirando la
    TFT, sin depender de la consola por SSH."""
    d = ImageDraw.Draw(img)
    sx, sy = scr
    txt = "raw %d,%d  ->  scr %d,%d" % (raw[0], raw[1], sx, sy)
    d.rectangle([30, 296, 450, 318], fill=(0, 0, 0))
    b = d.textbbox((0, 0), txt, font=F_SM)
    d.text((240 - (b[2] - b[0]) / 2, 298), txt, font=F_SM, fill=(255, 230, 0))
    if sx >= 0:
        d.line([sx - 14, sy, sx + 14, sy], fill=(255, 230, 0), width=2)
        d.line([sx, sy - 14, sx, sy + 14], fill=(255, 230, 0), width=2)
        d.ellipse([sx - 6, sy - 6, sx + 6, sy + 6], outline=(255, 230, 0), width=2)


def launch_retropie():
    es = shutil.which("emulationstation")
    if es:
        try:
            subprocess.run([es], check=False)
        except Exception as e:
            print("RetroPie error:", e, flush=True)


def main():
    fb = FrameBuffer()
    st = State()
    start_mqtt(st)
    touch = Touch(fb.w, fb.h)
    threading.Thread(target=touch.run, daemon=True).start()

    screen = "menu"
    last_draw = 0.0
    last_min = ""
    pending = True                 # redibujar solo cuando hace falta
    while True:
        tap = touch.get_tap()
        if tap:
            _, btns = (draw_menu(st) if screen == "menu" else draw_panel(st))
            for rect, action in btns:
                if inside(rect, tap):
                    if action == "homelab":
                        screen = "panel"
                    elif action == "menu":
                        screen = "menu"
                    elif action == "retropie":
                        launch_retropie()
                        screen = "menu"
                    break
            pending = True

        with st.lock:              # ¿llegaron datos nuevos?
            if st.dirty:
                st.dirty = False
                pending = True

        m = time.strftime("%H:%M")  # ¿cambió el minuto del reloj?
        if m != last_min:
            last_min = m
            pending = True

        now = time.time()
        # Dibuja al instante si tocaste; si es solo un refresco de datos, lo
        # limita a ~3 Hz. En reposo NO dibuja: el SPI queda libre y el touch
        # se lee limpio (menos toques fantasma).
        if pending and (tap or now - last_draw >= 0.3):
            img, _ = (draw_menu(st) if screen == "menu" else draw_panel(st))
            draw_tap_overlay(img, touch.last_raw, touch.last_scr)   # ayuda de calibración
            fb.show(img)
            last_draw = now
            pending = False
        time.sleep(0.02)


if __name__ == "__main__":
    main()
