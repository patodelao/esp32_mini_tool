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
| `labo/sensor/<id>/rssi` | `"-58"` (dBm) | salud: señal WiFi | sí |
| `labo/sensor/<id>/uptime` | `"143"` (min) | salud: minutos encendido | sí |
| `labo/sensor/<id>/heap` | `"28.4"` (kB) | salud: RAM libre | sí |
| `labo/nodo/<id>/status` | `online` / `offline` | — (LWT) | sí |
| `labo/alerta/<id>` | JSON `{origen,nivel,msg}` | alertas de riego / sonda | no |

La telemetría de salud (cada 60 s) sale como sensores normales, así que el
minitool ya la grafica, le guarda récord del día y le aplica umbrales: avisa
solo si el WiFi baja de −85 dBm o si la RAM libre cae bajo 8 kB (fuga).

Aparece solo en las tools **Sensores** y **Nodos** del minitool. El estado usa
Last-Will: si el nodo cae, el broker publica `offline` por él.

## Qué escucha (config del minitool)

| Topic | Payload | Qué hace |
|---|---|---|
| `labo/config/<id>/suelo/umbral` | `"25"` (%) | riega bajo ese %. `0` = no vigilar |
| `labo/config/<id>/suelo/histeresis` | `"5"` (%) | margen para dar por recuperado |
| `labo/config/<id>/suelo/intervalo` | `"300"` (s) | cada cuánto mide el suelo (5 s – 1 h) |
| `labo/nodo/<id>/cmd` | `leer` | publica una lectura ya, sin esperar el ciclo |
| `labo/nodo/<id>/cmd` | `reset` | reinicia el nodo |
| `labo/nodo/<id>/cmd` | `cal on` / `cal off` | modo calibración (ver abajo) |

Los comandos salen de la tool **Control** del minitool (tabla `s_cmds[]`).

El **aire** se publica siempre cada 10 s (`PUBLICAR_MS`); el **suelo** tiene su
propio ritmo configurable, porque la humedad de la tierra cambia en horas: medir
cada varios minutos alcanza, energiza menos la sonda y ahorra tráfico. Ojo que
`SUELO_CONFIRMAR` cuenta lecturas, no tiempo: con intervalo de 5 min, una alerta
de suelo seco tarda 3 lecturas = 15 min en confirmarse.

### Modo calibración

`cal on` hace que el nodo mida cada segundo y publique el crudo del ADC en
`labo/sensor/<id>/suelo_raw`, así que aparece como un sensor más ("Crudo Pieza")
en la tool Sensores. Sirve para **recalibrar sin desmontar nada**: mirás el
número en el minitool con la tierra seca y con la tierra recién regada, y esos
son tus `SUELO_SECO_RAW` / `SUELO_AGUA_RAW` (que se suben por OTA).

Se apaga solo a los 30 min, para que no quede midiendo seguido por olvido.

Ambos los publica **retenidos** el editor de umbrales de la tool Sensores
(botón del engranaje), así que el nodo los recibe al conectar, incluso después
de un corte de luz. Hasta que lleguen usa `SUELO_UMBRAL_DEF` / `SUELO_HISTER_DEF`.

## Cómo alerta el suelo

El nodo decide solo (no depende de que el minitool esté encendido):

- **Confirmación:** necesita `SUELO_CONFIRMAR` (3) lecturas seguidas — 30 s — bajo
  el umbral antes de avisar. Un pico suelto del ADC no dispara nada.
- **Histéresis:** para volver a "normal" el suelo tiene que superar
  `umbral + histéresis`, así el aviso no parpadea alrededor del límite.
- **Recordatorio:** mientras siga seco repite el aviso cada 30 min.
- **Sonda desconectada:** si el crudo se pega al tope (`SUELO_RAW_DESCONECTADO`)
  la lectura no es útil, así que **deja de publicar el %** (un `0` falso
  dispararía la alerta de riego) y manda una alarma al bus.
- **Mediana** de 9 lecturas del ADC en vez de promedio: descarta picos aislados.

Cada sensor es opcional: los flags `ENABLE_DHT` / `ENABLE_SUELO` (ambos en 1 por
defecto) permiten un nodo solo-aire o solo-suelo sin tocar más código.

## Conexionado (NodeMCU v3)

| Sensor | Pin sensor | Pin NodeMCU | Nota |
|---|---|---|---|
| DHT22 (aire) | VCC / GND / DATA | 3V3 / GND / **D2** | módulo 140C80 (pull-up incluido) |
| HW-103 (suelo) | VCC / GND / **A0** | **D5** / GND / **A0** | usar la salida analógica A0 del módulo, no D0 |

> ⚠️ El VCC del HW-103 va a **D5**, no a 3V3: el firmware trae
> `SUELO_VCC_PIN 14` y energiza la sonda solo el instante de medir, para que no
> se corroa. Si lo dejas en 3V3 el nodo lo detecta y avisa "sonda sin
> alimentación" (no publica humedad falsa). Para volver al cableado directo,
> pon `SUELO_VCC_PIN -1`.

> ⚠️ Alimentar todo a **3V3**, nunca a VU/VIN (5V): el GPIO y el ADC del ESP8266
> son de 3.3V y no toleran 5V.

### Calibración del sensor de suelo

El HW-103 es **resistivo**: seco da lectura ADC **alta**, en agua da **baja**.
Ajusta los dos valores en `main.cpp`:

```
#define SUELO_SECO_RAW  1023   // raw con la sonda en tierra seca
#define SUELO_AGUA_RAW   520   // raw con la sonda en agua
```

> ⚠️ Mide `SECO_RAW` con la sonda en **tierra seca**, no al aire. Al aire la
> lectura se clava en el tope del ADC (1023) y eso cuesta dos cosas: los
> porcentajes quedan comprimidos (la tierra seca real nunca llega al tope), y no
> queda margen para distinguir "tierra seca" de "sonda al aire" — con `SECO_RAW`
> en 1023, la detección de sonda desconectada queda deshabilitada sola, a
> propósito, porque sería indistinguible de una medición normal.

Si querés recuperar esa detección, vuelve a medir `SECO_RAW` con la sonda
enterrada en tierra bien seca (suele dar bastante menos de 1023) — se puede
hacer con el nodo instalado usando el modo calibración.

> 🔧 El sensor resistivo **se corroe** si está siempre energizado. Mejora futura
> fácil: alimentar su VCC desde un GPIO y encenderlo solo un instante antes de
> medir (extiende mucho su vida útil).

## Configuración (editar en `main.cpp`)

- `WIFI_SSID` / `WIFI_PASSWORD` — red 2.4 GHz (el ESP8266 no hace 5 GHz).
- `NODE_ID` — nombre con que aparece en las tools.
- `ENABLE_DHT` / `ENABLE_SUELO` — activar/desactivar cada sensor.
- `SUELO_SECO_RAW` / `SUELO_AGUA_RAW` — calibración del suelo.
- `SUELO_UMBRAL_DEF` / `SUELO_HISTER_DEF` — riego por defecto (los pisa el minitool).
- `SUELO_CONFIRMAR` — lecturas seguidas para confirmar seco/recuperado.
- `SUELO_RAW_DESCONECTADO` — crudo desde el cual se asume sonda al aire.

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

### Actualizar por WiFi (OTA), ya instalado

El nodo levanta ArduinoOTA con el hostname `pieza` y la clave `OTA_PASSWORD`.
Una vez montado no hace falta bajarlo para reflashearlo:

```bash
pio run --target upload --upload-port pieza.local
```

Si `pieza.local` no resuelve, usa la IP (la imprime al arrancar, y el minitool
la muestra en la tool Nodos). Cambia `OTA_PASSWORD` antes de instalarlo: con
ella cualquiera en tu red puede reflashear el nodo.

## Antes de dejarlo en su sitio definitivo

1. **Calibra en la maceta real**, no al aire: `cal on` desde la tool Control,
   mira "Crudo Pieza" en la tool Sensores con la tierra seca y recién regada, y
   pon esos números en `SUELO_SECO_RAW` / `SUELO_AGUA_RAW`. Los valores actuales
   (1024 / 520) son de prueba al aire, no de tu tierra. Esto se puede rehacer
   cuando quieras: el crudo llega por MQTT y el firmware se sube por OTA.
2. **Mira el RSSI en la posición final** (tool Sensores → `Wifi Pieza`). Bajo
   −85 dBm vas a tener cortes; mueve el nodo o el router antes de fijarlo.
3. **Mueve el VCC del HW-103 de 3V3 a D5** — el firmware ya viene con
   `SUELO_VCC_PIN 14`. Es lo único de esta lista que después no se puede
   cambiar por OTA, y es lo que hace que la sonda dure meses en vez de semanas.
4. **Cambia `OTA_PASSWORD`** y verifica que una subida OTA funcione **antes** de
   cerrar la caja.
5. Deja `CALIBRAR_SUELO 0` y `LIMPIAR_TOPICS_VIEJOS 0` (ya están así).
6. Sella con termorretráctil la unión cable–sonda: se corroe justo en la línea
   de la tierra húmeda.

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
