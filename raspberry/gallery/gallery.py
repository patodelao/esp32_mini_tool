#!/usr/bin/env python3
"""
gallery.py — Galería web de las fotos del nodo cam.

Sirve las fotos que junta cam_capture.py en ~/homelab/fotos/ para poder verlas
desde el navegador sin entrar por SSH. Solo biblioteca estándar (sin Flask ni
Pillow): en la Pi no hay que instalar nada.

  - /                muestra la grilla, más nuevas primero, paginada.
  - /fotos/<nombre>  sirve un JPEG puntual (con el nombre saneado).

Corre como servicio systemd (homelab-gallery). El firewall (ufw) limita el
puerto a la LAN, igual que el resto de los servicios de la Pi.
"""
import html
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs, quote

FOTOS_DIR = os.path.expanduser("~/homelab/fotos")
PORT = 8088
PAGE_SIZE = 60
# cam_capture.py nombra las fotos como cam-YYYYmmdd-HHMMSS.jpg
NAME_RE = re.compile(r"^cam-(\d{8})-(\d{6})\.jpg$")


def listar_fotos():
    """Nombres de foto, más nuevas primero. Como el nombre empieza con la
    fecha, ordenar por nombre es ordenar por tiempo."""
    try:
        nombres = [f for f in os.listdir(FOTOS_DIR) if NAME_RE.match(f)]
    except FileNotFoundError:
        return []
    return sorted(nombres, reverse=True)


def fecha_legible(nombre):
    m = NAME_RE.match(nombre)
    if not m:
        return nombre
    d, t = m.group(1), m.group(2)
    return "%s-%s-%s %s:%s:%s" % (d[0:4], d[4:6], d[6:8], t[0:2], t[2:4], t[4:6])


PAGINA = """<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Galería cam — RetroPi</title>
<style>
  :root {{ color-scheme: dark; }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; background: #14161a; color: #e6e6e6;
         font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; }}
  header {{ padding: 16px 20px; background: #1c1f26; position: sticky; top: 0;
           border-bottom: 1px solid #2a2e37; }}
  h1 {{ margin: 0; font-size: 18px; }}
  .sub {{ color: #9aa0aa; font-size: 13px; margin-top: 4px; }}
  .grid {{ display: grid; gap: 10px; padding: 16px 20px;
          grid-template-columns: repeat(auto-fill, minmax(180px, 1fr)); }}
  figure {{ margin: 0; background: #1c1f26; border: 1px solid #2a2e37;
           border-radius: 8px; overflow: hidden; }}
  figure img {{ width: 100%; height: 150px; object-fit: cover; display: block;
               background: #0d0f12; }}
  figcaption {{ padding: 6px 8px; font-size: 12px; color: #9aa0aa; }}
  .nav {{ display: flex; gap: 12px; justify-content: center; align-items: center;
         padding: 8px 20px 28px; }}
  .nav a {{ color: #e6e6e6; text-decoration: none; background: #2a2e37;
           padding: 8px 14px; border-radius: 6px; }}
  .nav span {{ color: #9aa0aa; font-size: 13px; }}
  .vacio {{ padding: 40px 20px; color: #9aa0aa; }}
  a.foto {{ color: inherit; text-decoration: none; }}
</style>
</head>
<body>
<header>
  <h1>Galería cam — RetroPi</h1>
  <div class="sub">{total} fotos · más nuevas primero{ultima}</div>
</header>
{cuerpo}
</body>
</html>
"""


def render(page):
    fotos = listar_fotos()
    total = len(fotos)
    if total == 0:
        cuerpo = '<div class="vacio">Todavía no hay fotos. Cuando el nodo cam ' \
                 'empiece a capturar, aparecerán acá.</div>'
        return PAGINA.format(total=0, ultima="", cuerpo=cuerpo)

    npages = (total + PAGE_SIZE - 1) // PAGE_SIZE
    page = max(0, min(page, npages - 1))
    lote = fotos[page * PAGE_SIZE:(page + 1) * PAGE_SIZE]

    tarjetas = []
    for nombre in lote:
        src = "/fotos/" + quote(nombre)
        cap = html.escape(fecha_legible(nombre))
        tarjetas.append(
            '<figure><a class="foto" href="{src}" target="_blank">'
            '<img loading="lazy" src="{src}" alt="{cap}">'
            '<figcaption>{cap}</figcaption></a></figure>'.format(src=src, cap=cap)
        )
    grid = '<div class="grid">' + "".join(tarjetas) + "</div>"

    nav = ['<div class="nav">']
    if page > 0:
        nav.append('<a href="/?p=%d">← más nuevas</a>' % (page - 1))
    nav.append('<span>página %d de %d</span>' % (page + 1, npages))
    if page < npages - 1:
        nav.append('<a href="/?p=%d">más viejas →</a>' % (page + 1))
    nav.append("</div>")

    ultima = " · última " + html.escape(fecha_legible(fotos[0]))
    return PAGINA.format(total=total, ultima=ultima, cuerpo=grid + "".join(nav))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # sin spamear el journal por cada imagen

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/" or u.path == "":
            try:
                p = int(parse_qs(u.query).get("p", ["0"])[0])
            except ValueError:
                p = 0
            self._html(render(p))
        elif u.path.startswith("/fotos/"):
            self._foto(u.path[len("/fotos/"):])
        else:
            self.send_error(404)

    def _html(self, texto):
        body = texto.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _foto(self, nombre):
        # Saneo: solo un basename que matchee el patrón de cam_capture.py.
        # Así no hay forma de salir de FOTOS_DIR (nada de '..' ni rutas).
        nombre = os.path.basename(nombre)
        if not NAME_RE.match(nombre):
            self.send_error(404)
            return
        ruta = os.path.join(FOTOS_DIR, nombre)
        if not os.path.isfile(ruta):
            self.send_error(404)
            return
        try:
            with open(ruta, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "max-age=86400")
        self.end_headers()
        self.wfile.write(data)


def main():
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print("galería en http://0.0.0.0:%d (fotos en %s)" % (PORT, FOTOS_DIR))
    srv.serve_forever()


if __name__ == "__main__":
    main()
