# esp8266_sensor

Nodo sensor basado en **ESP8266 (NodeMCU v3 / ESP-12E)**. Mide humedad y
temperatura del **aire** (DHT22) y humedad del **suelo** (módulo HW-103), y
publica todo por MQTT siguiendo las convenciones del minitool.

> ⚠️ **Toolchain distinto al resto del repo.** Los demás proyectos
> (`minitool`, `opendoor_alarm`) son ESP-IDF (`idf.py`, target esp32s3). Este
> proyecto usa **PlatformIO + Arduino** para ESP8266 (xtensa-lx106). La
> extensión ESP-IDF de VS Code **no** compila ESP8266.

## Qué publica

Al broker `broker.hivemq.com`, con `<id>` = `NODE_ID` (por defecto `pieza`):

| Topic | Payload | Sensor | Retain |
|---|---|---|---|
| `labo/sensor/<id>/temp` | `"23.4"` (°C) | aire (DHT22) | sí |
| `labo/sensor/<id>/hum` | `"51.2"` (%) | aire (DHT22) | sí |
| `labo/sensor/<id>/suelo` | `"45"` (%) | suelo (HW-103) | sí |
| `labo/nodo/<id>/status` | `online` / `offline` | — (LWT) | sí |

Aparece solo en las tools **Sensores** y **Nodos** del minitool. El estado usa
Last-Will: si el nodo cae, el broker publica `offline` por él.

Cada sensor es opcional: los flags `ENABLE_DHT` / `ENABLE_SUELO` (ambos en 1 por
defecto) permiten un nodo solo-aire o solo-suelo sin tocar más código.

## Conexionado (NodeMCU v3)

| Sensor | Pin sensor | Pin NodeMCU | Nota |
|---|---|---|---|
| DHT22 (aire) | VCC / GND / DATA | 3V3 / GND / **D2** | módulo 140C80 (pull-up incluido) |
| HW-103 (suelo) | VCC / GND / **A0** | 3V3 / GND / **A0** | usar la salida analógica A0 del módulo, no D0 |

> ⚠️ Alimentar todo a **3V3**, nunca a VU/VIN (5V): el GPIO y el ADC del ESP8266
> son de 3.3V y no toleran 5V.

### Calibración del sensor de suelo

El HW-103 es **resistivo**: seco/al aire da lectura ADC **alta**, en agua da
**baja**. El monitor imprime el valor crudo (`raw`); ajústalo en `main.cpp`:

```
#define SUELO_SECO_RAW  820   // raw con la sonda al aire
#define SUELO_AGUA_RAW  380   // raw con la sonda en agua
```

Mide primero al aire, anota el número; sumerge en agua, anota el otro.

> 🔧 El sensor resistivo **se corroe** si está siempre energizado. Mejora futura
> fácil: alimentar su VCC desde un GPIO y encenderlo solo un instante antes de
> medir (extiende mucho su vida útil).

## Configuración (editar en `main.cpp`)

- `WIFI_SSID` / `WIFI_PASSWORD` — red 2.4 GHz (el ESP8266 no hace 5 GHz).
- `NODE_ID` — nombre con que aparece en las tools.
- `ENABLE_DHT` / `ENABLE_SUELO` — activar/desactivar cada sensor.
- `SUELO_SECO_RAW` / `SUELO_AGUA_RAW` — calibración del suelo.

Otros ajustes en `platformio.ini`: `board = nodemcuv2` (vale para v2 y v3),
`monitor_speed = 115200`, y `upload_port` / `monitor_port` si el COM no se
autodetecta.

## Requisitos (una sola vez)

1. En VS Code, instala la extensión **PlatformIO IDE** (marketplace). Convive
   con tu extensión ESP-IDF sin problema.
2. Driver USB del NodeMCU: normalmente **CH340** (o CP2102).

## Compilar, subir y monitorear

Desde la barra inferior de VS Code (íconos de PlatformIO) o por CLI:

```bash
pio run                    # compilar
pio run --target upload    # subir al NodeMCU
pio device monitor         # monitor serie (115200)
```

Salida esperada:

```
=== ESP8266 sensor: aire (DHT22) + suelo (HW-103) + MQTT ===
Nodo: pieza
  aire  -> labo/sensor/pieza/temp | labo/sensor/pieza/hum (D2)
  suelo -> labo/sensor/pieza/suelo (A0)
WiFi: conectado. IP 192.168.x.x  RSSI -58 dBm
MQTT: conectado
Aire  -> T: 23.4 C   HR: 51.2 %
Suelo -> 45 % (raw 604)
```
