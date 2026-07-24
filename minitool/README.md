# minitool

Reloj-herramienta sobre **Waveshare ESP32-S3-Touch-LCD-1.28**: pantalla redonda
de 240×240 táctil, IMU de 6 ejes y Wi-Fi/BLE. Hace de **consola del home-lab**
(sensores, nodos, alertas, control) y de reloj con las funciones que se esperan
de uno: notificaciones del teléfono, podómetro, pantalla que se apaga y
despierta al moverlo.

> Este README reemplaza al del ejemplo `spi_lcd_touch` de Espressif, que venía
> de plantilla y describía un cableado que no corresponde a esta placa (acá el
> panel y el táctil vienen integrados).

## Compilar y flashear

ESP-IDF **v5.3.2**, target `esp32s3`. Desde PowerShell hay que fijar el
entorno a mano porque `export.ps1` toma un Python equivocado:

```bash
$env:IDF_TOOLS_PATH="C:/Users/alons/Espressif"; $env:PATH="C:/Users/alons/Espressif/python_env/idf5.3_py3.11_env/Scripts;" + $env:PATH; . "C:/Users/alons/esp/v5.3.2/esp-idf/export.ps1"
```

```bash
idf.py -p COM<x> flash monitor
```

> ⚠️ **No usar `erase-flash`.** La NVS guarda credenciales Wi-Fi, umbrales de
> sensores, récord del día, historial de alertas, pasos y ajustes de pantalla.
> Un `flash` normal no la toca.

En un checkout nuevo, antes del primer build: `idf.py set-target esp32s3`
(`sdkconfig.defaults` no fija el target y el default no entra en memoria).

## Cómo se navega

Sin botones físicos, todo es táctil. Vale la pena tener esto a mano:

| Gesto | Dónde | Qué hace |
|---|---|---|
| Tocar el **ícono** de la fila central | Menú | Abre esa herramienta |
| Arrastrar la barra inferior | Menú | Salta a esa posición de la lista |
| Deslizar a la **derecha** | En una tool | Vuelve al menú (o a Config, si se abrió desde ahí) |
| Deslizar **abajo** | Carátula | Abre el **panel rápido** (silencio, linterna, BT, brillo) |
| Deslizar **arriba** | Panel rápido | Lo cierra |
| Tocar la pantalla | Carátula | Vuelve al menú |
| Tocar la pantalla | Alarma sonando | La apaga |
| Tocar el gráfico | Sensores (detalle) | Alterna histórico reciente / 24 h |
| Pulsación **larga** en la papelera | Sensores (detalle) | Olvida el sensor (pide confirmar con una X) |

## Arquitectura

Dos capas, y la separación importa: **los servicios corren siempre**, las
herramientas son solo vistas.

### Servicios (segundo plano)

| Módulo | Qué hace |
|---|---|
| `mqtt_hub` | Un solo cliente MQTT compartido. Los subsistemas se enganchan con un filtro de topic y un callback |
| `sensor_service` | Lecturas de `labo/sensor/#`: último valor, histórico corto, promedio horario de 24 h y récord del día (todo persistido) |
| `sensor_alert` | Umbrales por sensor con histéresis, anti-rebote y aviso de sensor mudo. Ver abajo |
| `fleet_service` | Nodos vistos en `labo/nodo/+/status` y sus IPs |
| `self_node` | El propio reloj publicándose como un nodo más (estado, IP, rssi/heap/uptime) |
| `alert_service` | Bus de alertas `labo/alerta/#` → notificaciones, y estado de la puerta del refri |
| `weather_service` | Clima con caché. Prefiere el que manda el teléfono; solo descarga de internet si no hay |
| `pedometer_service` | Pasos del día por acelerómetro, con historial de 7 días y meta |
| `alarm_clock` | Alarmas diarias. Suena con cualquier tool abierta y con la pantalla apagada |
| `ble_notify` | Notificaciones del teléfono por BLE (Gadgetbridge), y control de música / buscar teléfono |
| `ui_notify` | Toasts + historial persistido de las últimas 20 notificaciones |
| `ui_power` | Apagado de pantalla, despertar por movimiento, brillo y modo noche |
| `ui_quick` | Panel rápido de la carátula |

### Herramientas

Cada una es un `const tool_t` en `main/tools/` con `open`/`close`, registrado
en `tools.c`. Para agregar una: implementar el `.c`, declararla `extern` en
`tools.c` y sumarla al array.

El flag `.hidden` la saca del menú principal: así las de comunicaciones
(Wi-Fi, Redes, Info de red, QR WiFi, Bluetooth) viven dentro de **Config** sin
duplicar código, y se abren con `ui_menu_open_tool()`.

## Sensores y alertas

La tool **Sensores** abre en una lista con todos (hoy 13, entre la pieza, el
refri y el propio reloj) y al tocar uno se ve su detalle: valor, récord del
día, gráfico e histórico de 24 h.

El motor de umbrales (`sensor_alert.c`) da, por sensor:

- **banda min/max** con cada límite activable por separado,
- **histéresis** para volver a normal sin que el aviso parpadee,
- **anti-rebote**: dos lecturas seguidas fuera de rango antes de avisar, para
  que un pico del ADC no dispare nada,
- **recordatorio** cada 30 min mientras siga fuera,
- **aviso de sensor mudo**, con el límite derivado del intervalo de muestreo, y
  agrupado **por nodo**: si se cae el nodo de la pieza no salen 6 avisos, sale
  uno. Y no se avisa de un nodo que se declaró offline, porque eso es que está
  apagado, no que falle.

Los valores por defecto salen de la magnitud (`suelo`, `temp`, `rssi`…) y se
editan desde el engranaje, que además publica la config al nodo cuando
corresponde. Ver [README raíz](../README.md) para la convención de topics.

## Teléfono (Gadgetbridge)

El reloj se anuncia por BLE como **"Bangle.js mini"** —el prefijo es lo que lo
hace reconocible— y habla el protocolo de Bangle.js sobre Nordic UART:

- **recibe** notificaciones, llamadas, la hora y el clima;
- **envía** control de música y "encontrar mi teléfono" (tool **Teléfono**).

Requiere [Gadgetbridge](https://f-droid.org/packages/nodomain.freeyourgadget.gadgetbridge/)
desde F-Droid (no está en Play Store). En Xiaomi/MIUI hay que darle
**autoinicio** y **batería sin restricciones**, o deja de reenviar a los pocos
minutos.

## Pantalla y energía

- Se apaga sola tras el tiempo configurado (45 s por defecto).
- Despierta al **tocarla**, al **moverla** o cuando entra una notificación. El
  movimiento se mide contra una línea base del reposo, no contra cero: el
  giroscopio quieto marca su propio sesgo, y restarlo permite umbrales mucho
  más chicos sin despertares falsos.
- **Modo noche**: en la franja configurada las notificaciones van solo al
  historial y la pantalla enciende al 15 %. Las críticas pasan igual.
- La linterna y la alarma del temporizador **inhiben** el apagado: son cosas
  que se miran sin tocar la pantalla.

## Particiones

`partitions.csv`: 3 MB de app + 1 MB de SPIFFS sobre 16 MB de flash. Quedan
~12 MB sin particionar, disponibles si algún día hace falta.
