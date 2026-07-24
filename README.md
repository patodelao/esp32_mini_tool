# esp32_mini_tool

Home-lab casero: un reloj-herramienta con pantalla táctil que hace de **consola**
del sistema, y nodos ESP que miden y actúan. Todo se comunica por **MQTT**, sin
servidor propio: los nodos publican, el minitool muestra y configura.

## Proyectos

| Carpeta | Qué es | Toolchain |
|---|---|---|
| [`minitool/`](minitool) | ESP32-S3 con pantalla redonda 240×240 táctil. La consola del home-lab y reloj: sensores, nodos, alertas, control, notificaciones del teléfono. | ESP-IDF v5.3.2, LVGL 8 |
| [`esp8266_sensor/`](esp8266_sensor) | Nodo `pieza`: aire (DHT22) + suelo (HW-103), con riego vigilado y actualización OTA. | PlatformIO + Arduino |
| [`opendoor_alarm/`](opendoor_alarm) | Nodo `refri`: alarma de puerta abierta, con actualización OTA. | ESP-IDF |
| [`esp32cam/`](esp32cam) | Nodo `cam` (AI-Thinker ESP32-CAM). Por ahora solo base de red + OTA; la cámara viene después. | ESP-IDF v5.3.2 |

**Los cuatro se actualizan por Wi-Fi.** El de la pieza con ArduinoOTA; el
refri, el minitool y la cámara con un POST HTTP a su propio servidor. Ninguno
necesita cable salvo el primer flasheo (o un cambio en la tabla de
particiones).

> ⚠️ Son **tres toolchains distintas** (ESP-IDF para minitool/refri/cam,
> PlatformIO+Arduino para el ESP8266). La extensión ESP-IDF de VS Code no
> compila el ESP8266, y PlatformIO no compila los proyectos ESP-IDF.

## Cómo se hablan: convención de topics

Broker público `broker.hivemq.com` (se cambia en `minitool/main/mqtt_hub.c`).
Cualquier equipo que respete estos topics aparece solo en las tools, sin tocar
código del minitool:

| Patrón | Sentido | Quién lo usa |
|---|---|---|
| `labo/sensor/<nodo>/<magnitud>` | nodo → minitool | tool **Sensores**: valor, gráfico, récord del día y umbrales |
| `labo/nodo/<nodo>/status` | nodo → minitool | tool **Nodos**: online/offline (con Last-Will) |
| `labo/nodo/<nodo>/ip` | nodo → minitool | tool **Nodos**: IP para actualizar por OTA |
| `labo/nodo/<nodo>/cmd` | minitool → nodo | tool **Control**: órdenes (`leer`, `reset`, …) |
| `labo/nodo/<nodo>/<estado>` | nodo → minitool | estados de texto, no numéricos (p. ej. `refri/puerta` → `ABIERTO`) |
| `labo/config/<nodo>/...` | minitool → nodo | configuración **retenida** (umbrales, intervalos) |
| `labo/alerta/<origen>` | cualquiera → minitool | bus de alertas: JSON `{origen,nivel,msg}` → notificación flotante |

Toda notificación —del bus, de los umbrales locales o de la tool Control— queda
además registrada en la tool **Alertas**, con su hora. Un toast dura unos
segundos: si no estabas mirando la pantalla, ahí es donde lo encontrás. En la
franja de **modo noche** las notificaciones van solo al historial, sin
interrumpir; las críticas pasan igual.

El minitool **también se publica a sí mismo** como nodo (`labo/nodo/minitool/…`
y `labo/sensor/minitool/…`), así que aparece en sus propias tools Nodos y
Sensores, con last-will incluido. Era el único equipo que, si se colgaba, no
quedaba registrado en ningún lado.

El payload de un sensor es simplemente el número como texto (`"23.5"`). La
magnitud (`temp`, `hum`, `suelo`, `rssi`…) define la unidad y los umbrales por
defecto; ver `minitool/main/sensor_service.c` y `sensor_alert.c`.

## Dónde vive la lógica de alertas

Está repartida a propósito, y conviene entender el reparto antes de tocar algo:

- **En el nodo** vive lo que tiene que funcionar aunque el minitool esté
  apagado. El riego es el caso claro: el ESP8266 compara contra su umbral y
  publica la alerta al bus por su cuenta. El minitool solo le manda la
  configuración, retenida, para que la reciba incluso tras un corte de luz.
- **En el minitool** (`sensor_alert.c`) vive todo lo demás: umbrales de
  cualquier sensor, histéresis, anti-rebote, aviso de sensor mudo y
  recordatorios. Se configura desde la pantalla y se guarda en NVS.

Para no duplicar avisos, una regla puede marcarse `remote_lo`: el minitool
sigue el estado para colorear la UI pero **no** emite su propio toast, porque el
nodo ya publicó el suyo.

## Por dónde empezar

- Armar o replicar un nodo sensor: [`esp8266_sensor/README.md`](esp8266_sensor/README.md)
  — conexionado, calibración, OTA y checklist de instalación.
- Entender o modificar el reloj: [`minitool/README.md`](minitool/README.md) —
  arquitectura (servicios vs. herramientas), cómo agregar una tool, el mapa de
  gestos y los detalles de compilación.
- El nodo del refri: [`opendoor_alarm/README.md`](opendoor_alarm/README.md).
- El nodo de la cámara: [`esp32cam/README.md`](esp32cam/README.md).

## Qué hay funcionando hoy

| Nodo | Publica | Actualización |
|---|---|---|
| `pieza` | temp, hum, suelo, rssi, uptime, heap | OTA (ArduinoOTA) |
| `refri` | puerta, abierta_seg, rssi, uptime, heap | OTA (`curl` a `http://<ip>/update`) |
| `minitool` | rssi, uptime, heap | OTA (`curl` a `http://<ip>/update`) |
| `cam` | rssi, uptime, heap | OTA (`curl` a `http://<ip>/update`) |

Cada nodo publica además su `status` (con last-will) y su `ip`, que la tool
**Nodos** muestra para poder actualizarlo.

El minitool sirve también un **panel web** en `http://<ip>/`: una tarjeta por
sensor con su valor, la curva de las últimas 24 h y el récord del día,
agrupadas por nodo, más los nodos, las alertas y el registro. Todo en el
navegador del teléfono, sin tener el reloj en la mano. Los gráficos son SVG
generado en el propio ESP32, sin librerías ni JavaScript.

Además guarda en su flash una foto de todos los sensores cada media hora y la
entrega en `http://<ip>/csv` — entre 3 y 6 semanas de historia para abrir en
una planilla. Es la única memoria larga del sistema: los nodos publican y
olvidan.

La telemetría de salud sale por los topics de sensores a propósito: así hereda
gratis el gráfico, el récord del día, el histórico de 24 h y los umbrales. El
reloj se avisa a sí mismo si su Wi-Fi cae de −85 dBm o si su RAM libre baja de
8 kB.
