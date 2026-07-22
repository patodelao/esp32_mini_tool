# esp8266_sensor

Nodo sensor basado en **ESP8266 (NodeMCU v3 / ESP-12E)**.

> ⚠️ **Toolchain distinto al resto del repo.** Los demás proyectos
> (`minitool`, `opendoor_alarm`, `smart_lamp`) son ESP-IDF (`idf.py`, target
> esp32s3). Este proyecto usa **PlatformIO + Arduino** para ESP8266
> (xtensa-lx106). La extensión ESP-IDF de VS Code **no** compila ESP8266.

## Estado actual

Hola mundo: parpadea el LED integrado y publica un contador por el puerto
serie. Sirve para validar que el toolchain compila, sube y monitorea antes de
agregar sensores o MQTT.

## Requisitos (una sola vez)

1. En VS Code, instala la extensión **PlatformIO IDE** (marketplace).
   Convive con tu extensión ESP-IDF sin problema.
2. Driver USB del NodeMCU: normalmente **CH340** (o CP2102). Instálalo si
   Windows no reconoce el puerto COM.

## Compilar, subir y monitorear

Desde la barra inferior de VS Code (íconos de PlatformIO) o por CLI:

```bash
pio run                    # compilar
pio run --target upload    # subir al NodeMCU
pio device monitor         # monitor serie (115200)
```

Salida esperada en el monitor:

```
=== ESP8266 sensor: hola mundo ===
Chip ID: XXXXXXXX  |  Flash: 4194304 bytes
Hola mundo #0  (uptime 1 s)
Hola mundo #1  (uptime 2 s)
...
```

## Configuración

- **Placa:** `board = nodemcuv2` en `platformio.ini` (vale para NodeMCU v2 y v3,
  ambos ESP-12E, 4MB flash).
- **Puerto COM:** PlatformIO lo autodetecta. Si falla, descomenta
  `upload_port` / `monitor_port` en `platformio.ini` con tu COM.
- **LED integrado:** GPIO2 (D4), activo en bajo.

## Próximos pasos

Integrar con las convenciones MQTT del home-lab (broker `broker.hivemq.com`):
publicar lecturas en `labo/sensor/<id>` y estado en `labo/nodo/<id>/status`,
para que aparezca en las tools del minitool. Librerías típicas: `PubSubClient`
(MQTT) y la del sensor concreto (ej. `DHT`), agregadas en `lib_deps` de
`platformio.ini`.
