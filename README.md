# esp32_mini_tool

Home-lab casero: un reloj-herramienta con pantalla táctil que hace de **consola**
del sistema, y nodos ESP que miden y actúan. Todo se comunica por **MQTT**, sin
servidor propio: los nodos publican, el minitool muestra y configura.

## Proyectos

| Carpeta | Qué es | Toolchain |
|---|---|---|
| [`minitool/`](minitool) | ESP32-S3 con pantalla redonda 240×240 táctil. La consola: menú de herramientas (sensores, nodos, control, clima, reloj, podómetro…). | ESP-IDF v5.3.2, LVGL 8 |
| [`esp8266_sensor/`](esp8266_sensor) | Nodo sensor: aire (DHT22) + suelo (HW-103), con riego vigilado y actualización OTA. | PlatformIO + Arduino |
| [`opendoor_alarm/`](opendoor_alarm) | Alarma de puerta abierta del refrigerador. | ESP-IDF |

> ⚠️ Son **tres toolchains distintas**. La extensión ESP-IDF de VS Code no
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
| `labo/config/<nodo>/...` | minitool → nodo | configuración **retenida** (umbrales, intervalos) |
| `labo/alerta/<origen>` | cualquiera → minitool | bus de alertas: JSON `{origen,nivel,msg}` → notificación flotante |

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
- Compilar el minitool: ESP-IDF v5.3.2, target `esp32s3`. En un checkout nuevo,
  correr `idf.py set-target esp32s3` **antes** del primer `idf.py build`
  (`sdkconfig.defaults` no fija el target y el default no entra en memoria).
