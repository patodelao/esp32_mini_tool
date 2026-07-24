# esp32cam — nodo `cam` (AI-Thinker ESP32-CAM)

El cuarto nodo del home-lab: una cámara con foto por HTTP, integrada al mismo
bus MQTT que los demás y **actualizable por Wi-Fi**.

## La foto

`http://<ip>/foto.jpg` devuelve una captura JPEG desde cualquier navegador de
la red. Con PSRAM sale en SVGA (800×600); si por lo que sea no hay PSRAM, se
degrada a QVGA en vez de fallar. El sensor entrega JPEG directo, así que el
frame se manda tal cual, sin recomprimir.

La cámara arranca al boot pero **no es crítica**: si falla (placa sin sensor,
PSRAM que no monta), el nodo sigue vivo como nodo OTA y `/foto.jpg` responde
503. Nunca un cuelgue de arranque que obligue a volver al cable.

## Por qué el OTA es el corazón del proyecto

El ESP32-CAM no tiene USB. Se flashea con un adaptador USB-TTL puenteando
`GPIO0` a masa y apretando reset — un baile molesto. Por eso el primer flasheo
por cable fue el **único**: de ahí en más el binario entra por Wi-Fi, igual que
el nodo del refri.

## Qué hace

- Se conecta al Wi-Fi y al broker MQTT (`broker.hivemq.com`).
- Sirve la foto en `http://<ip>/foto.jpg`.
- Se anuncia y publica telemetría con las convenciones del minitool, así que
  aparece solo en las tools **Nodos** y **Sensores**:

  | Topic | Contenido |
  |---|---|
  | `labo/nodo/cam/status` | `online`/`offline` (retenido, con last-will) |
  | `labo/nodo/cam/ip` | IP para llegarle |
  | `labo/nodo/cam/cmd` | comandos: `reset`, `leer` |
  | `labo/sensor/cam/rssi` | dBm |
  | `labo/sensor/cam/uptime` | minutos |
  | `labo/sensor/cam/heap` | kB libres |
  | `labo/alerta/cam` | JSON `{origen:"Camara",nivel,msg}` |

- Sirve el **servidor de OTA** en `http://<ip>/` y acepta el firmware por POST.
- **LED de vida** (rojo de a bordo, `GPIO33`): parpadea mientras busca red,
  queda fijo cuando MQTT está conectado. En una placa sin consola es la única
  señal de que está viva. No se usa el `GPIO4` (el flash blanco encandila).

## Primer flasheo (por cable, una vez)

ESP-IDF **v5.3.2**, target `esp32` (el AI-Thinker es ESP32 clásico, no S3).

```bash
copy esp32cam\main\secrets.h.example esp32cam\main\secrets.h
```

Completá tu red en `secrets.h` (está en `.gitignore`). Después, con el
adaptador USB-TTL y `GPIO0` a masa:

```bash
idf.py -p COM<x> flash monitor
```

En un checkout nuevo, antes del primer build: `idf.py set-target esp32`.

## Todas las veces siguientes (por Wi-Fi)

La IP la muestra la tool **Nodos** del minitool, o el propio `http://<ip>/`.

```bash
curl -X POST --data-binary @build/esp32cam.bin "http://<ip>/update?key=cam-ota"
```

La clave está en `OTA_PASSWORD` (`main/secrets.h`). Viaja en la URL sobre HTTP
plano: alcanza para que nadie de la red reflashee la cámara por accidente, no
contra alguien que espíe el tráfico.

El firmware se escribe en la ranura que **no** está corriendo (`partitions.csv`
define `ota_0`/`ota_1` de 1.87 MB cada una); el arranque se cambia recién
cuando `esp_ota_end()` valida el binario. Una subida cortada no rompe nada:
sigue arrancando el viejo.

## Cámara: detalles

- Componente `espressif/esp32-camera`, declarado en `main/idf_component.yml`
  (el component manager lo baja solo en el primer build — necesita internet esa
  vez). **No** va en el `REQUIRES` del `CMakeLists`: listarlo a mano rompe la
  resolución del nombre.
- Mapa de pines del AI-Thinker en `camera.c`; si algún día se usa otra placa,
  cambia solo ese bloque.
- La PSRAM se habilita en `sdkconfig.defaults` con `CONFIG_SPIRAM_IGNORE_NOTFOUND`
  como red de seguridad: si no monta, el boot no se aborta.

## Particiones

Dos ranuras de app de 1.87 MB + NVS, sobre los 4 MB de la placa. Con la cámara,
el binario ronda ~1.2 MB; sigue quedando margen holgado en la ranura.

## Lo que viene

- Disparo por **detección de movimiento** publicado al bus de alertas
  (`labo/alerta/cam`), que el minitool ya sabe mostrar.
- Guardar la última foto de una alerta, o servir un stream MJPEG en vez de una
  sola foto.
