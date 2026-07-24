# esp32cam — nodo `cam` (AI-Thinker ESP32-CAM)

El cuarto nodo del home-lab. **Esta primera etapa NO tiene cámara todavía**:
es solo la base de red y —sobre todo— la **actualización por Wi-Fi**, para no
tener que volver a flashear por cable.

## Por qué se arranca por el OTA

El ESP32-CAM no tiene USB. Se flashea con un adaptador USB-TTL puenteando
`GPIO0` a masa y apretando reset — un baile molesto que no conviene repetir. La
idea de esta etapa es dejar ese baile hecho **una sola vez**: con este firmware
base andando, el primer flasheo por cable es el último. De ahí en más el
binario (ya con cámara) entra por Wi-Fi, igual que el nodo del refri.

## Qué hace hoy

- Se conecta al Wi-Fi y al broker MQTT (`broker.hivemq.com`).
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

## Particiones

Dos ranuras de app de 1.87 MB + NVS, sobre los 4 MB de la placa. El firmware de
esta etapa pesa ~0.9 MB (53% libre en la ranura); cuando entre la cámara
(`esp32-camera` + JPEG) va a crecer, pero queda margen holgado.

## Lo que viene

- **Cámara**: `esp32-camera`, pines del AI-Thinker, y habilitar la PSRAM en
  `sdkconfig.defaults` (los frame buffers la necesitan; hoy está apagada a
  propósito, porque prenderla mal es una fuente clásica de cuelgues de arranque).
- Foto bajo demanda vía `cmd`, o servida en `http://<ip>/foto.jpg`, y quizás un
  disparo por detección de movimiento publicado al bus de alertas.
