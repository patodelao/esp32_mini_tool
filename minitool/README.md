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

### Credenciales

La red se carga desde la tool **Red** y queda en NVS; eso es lo que manda. Como
respaldo, `main/secrets.h` lleva las credenciales compiladas, igual que los
otros dos nodos:

```bash
copy minitool\main\secrets.h.example minitool\main\secrets.h
```

Está en `.gitignore`; el `.example` sí se versiona. Sin él el proyecto compila
(avisa con un `#warning`) pero arranca con relleno que no conecta a nada.

Sirve para un caso concreto: si la NVS queda en blanco, el reloj vuelve solo a
la red. Antes no lo hacía y era **el estado más difícil de diagnosticar del
sistema** — la pantalla se ve perfecta, el menú responde, pero no hay Wi-Fi, ni
MQTT, ni panel web, y nada en pantalla dice por qué. Dos causas se juntaban: sin
credenciales guardadas quedaban las de relleno, y la intención de conectarse
también salía de la NVS, así que sin ella el reloj ni siquiera lo intentaba.
Hoy, si esa clave no está, se asume que sí hay que conectarse; solo un
"desconectar" explícito lo deja apagado.

## Cómo se navega

Sin botones físicos, todo es táctil. Vale la pena tener esto a mano:

| Gesto | Dónde | Qué hace |
|---|---|---|
| Tocar una tarjeta | Menú | Abre esa herramienta |
| Deslizar a la **derecha** | En una tool | Vuelve al menú (o a Config, si se abrió desde ahí) |
| Deslizar **abajo** | Carátula | Abre el **panel rápido** (silencio, linterna, BT, brillo) |
| Deslizar **arriba** | Panel rápido | Lo cierra |
| Tocar la pantalla | Carátula | Vuelve al menú |
| Tocar la pantalla | Alarma sonando | La apaga |
| Tocar el gráfico | Sensores (detalle) | Cicla: reciente → 24 h → 7 días (min/max) |
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
| `web_ui` | Servidor HTTP: panel del home-lab para el navegador del teléfono y actualización del propio reloj por Wi-Fi. Ver abajo |
| `datalog` | Registro histórico en SPIFFS: una foto de todos los sensores cada 30 min, descargable en CSV. Ver abajo |
| `weather_service` | Clima con caché. Prefiere el que manda el teléfono; solo descarga de internet si no hay |
| `pedometer_service` | Pasos del día por acelerómetro, con historial de 7 días y meta |
| `alarm_clock` | Alarmas diarias. Suena con cualquier tool abierta y con la pantalla apagada |
| `ble_notify` | Notificaciones del teléfono por BLE (Gadgetbridge), y control de música / buscar teléfono |
| `ui_notify` | Toasts + historial persistido de las últimas 20 notificaciones |
| `ui_power` | Apagado de pantalla, despertar por movimiento, brillo y modo noche |
| `ui_quick` | Panel rápido de la carátula |

### Estética

`ui_theme.h` define **una vez** cada rol de color (`UI_OK`, `UI_ALERT`,
`UI_CARD`, `UI_TITLE`…) y las piezas repetidas (título de tool, tarjeta,
píldora). Los nombres son semánticos a propósito: si mañana cambia el verde,
cambia en un solo lugar y nada queda a medio camino.

El menú, Config, Sensores y Carátula comparten el mismo lenguaje: lista de
píldoras con el ícono en un chip de color, curvada siguiendo el borde circular
y con snap al centro.

En el menú, además:

- **La píldora del centro crece.** El foco se lee sin tener que interpretar el
  degradé de opacidad: hay una elegida, no una lista que se desvanece.
- **El marco del borde es la barra de scroll.** Una barra recta en una pantalla
  redonda queda mal y ocupa lugar; el borde ya es circular, así que el
  recorrido se dibuja ahí y toma el color de la tool en foco.

### Herramientas

Cada una es un `const tool_t` en `main/tools/` con `open`/`close`, registrado
en `tools.c`. Para agregar una: implementar el `.c`, declararla `extern` en
`tools.c` y sumarla al array.

El flag `.hidden` la saca del menú principal: así las de comunicaciones
(Wi-Fi, Redes, Info de red, QR WiFi, Bluetooth) viven dentro de **Config** sin
duplicar código, y se abren con `ui_menu_open_tool()`.

## Sensores y alertas

La tool **Sensores** abre en una lista con todos (hoy 13, entre la pieza, el
refri y el propio reloj) y al tocar uno se ve su detalle. El gráfico tiene tres
escalas que se ciclan tocándolo:

| Modo | Qué muestra | Para qué |
|---|---|---|
| reciente | últimas 30 lecturas | qué está pasando ahora |
| 24 h | un promedio por hora | la curva del día (p. ej. cuánto tarda en secarse la maceta) |
| 7 días | banda min–max de cada día | **a cuánto llegó a bajar el suelo esta semana**, que es lo que hace falta para elegir el umbral de riego con datos |

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

## Estado del sistema

Config → **Estado**. No informa: diagnostica. Cada línea responde una pregunta
que uno se hace justo en ese momento:

| Línea | Qué responde |
|---|---|
| **Wi-Fi** | A qué red apunta, y si está *buscando* o *apagado* |
| **MQTT** | Si hay red pero el broker no contesta |
| **Hora** | Si sincronizó (sin hora no hay alarmas, récords ni registro) |
| **Datos** | Si el registro histórico está guardando |

La distinción entre *buscando* y *apagado* es la que faltaba: "sin conexión"
tapaba las dos, y son problemas opuestos. Con el SSID al lado se ve enseguida
si está intentando entrar a la red equivocada.

En la carátula, el icono de Wi-Fi **ya no desaparece** cuando no hay red:
queda rojo si busca y no llega, gris si está apagado a propósito. Antes se
escondía, y una carátula sin iconos se ve igual de prolija que una conectada —
el reloj podía estar incomunicado durante horas sin que nada lo delatara.

## El dato de la carátula

Abajo a la izquierda hay un hueco que mostraba siempre los pasos del día. El
valor no estaba en los pasos sino en el hueco: es el único lugar donde se lee
un dato **sin tocar la pantalla**. Qué dato merece ese lugar depende de cada
uno, así que ahora se elige en **Config → Carátula**: los pasos, o cualquier
sensor del home-lab.

La lista se arma en el momento con los sensores vivos, así que un nodo nuevo
aparece solo. El valor elegido se pinta con el mismo código de color del resto
del sistema (rojo fuera de umbral, gris si el dato está viejo). Si el sensor
deja de existir se muestra `--` en vez de esconderse: haberlo elegido y que no
llegue también es información.

A la derecha sigue la humedad del suelo, que se pone roja cuando hay que regar.

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

## Panel web y actualización por Wi-Fi

El reloj levanta un servidor HTTP (`web_ui.c`) apenas hay red. La IP la muestra
la tool **Red**, y también la publica en `labo/nodo/minitool/ip`.

### Ver el home-lab desde el teléfono

`http://<ip>/` devuelve un panel con **una tarjeta por sensor**: nombre, valor
grande con el mismo color de estado que en pantalla, la **curva de las últimas
24 h** y el récord del día. Debajo, los nodos con su IP, las últimas alertas y
el estado del registro. Se refresca solo cada 30 s. La pantalla del reloj mide
240×240 y hay que tenerlo en la mano; en el navegador entra todo junto.

Las tarjetas van **agrupadas por nodo**, que es como uno los piensa: la
pregunta es "¿cómo está la pieza?", no "¿cómo está `pieza/temp`?". Con 13
sensores de tres equipos, una lista corrida no se lee.

Las curvas son **SVG generado en el ESP32**, sin librerías ni JavaScript: una
polilínea escalada al alto de la caja. Los datos salen de la historia horaria
que `sensor_service` ya guarda en NVS — no del CSV de la flash, que son
cientos de kB y parsearlos por cada visita tardaría una eternidad. Una serie
plana (el suelo que no se movió en todo el día) recibe aire artificial para no
dividir por cero ni quedar pegada al borde.

Un número solo no dice nada: 18.9 °C puede ser una pieza estable o una que
viene cayendo hace seis horas, y son cosas distintas. La curva contesta eso de
un vistazo.

### Actualizar sin cable

```bash
curl -X POST --data-binary @build/spi_lcd_touch.bin "http://<ip>/update?key=minitool-ota"
```

La clave está en `WEB_OTA_KEY` (`main/web_ui.c`). Va en la URL sobre HTTP
plano: alcanza para que nadie de la red lo reflashee por accidente, no contra
alguien que espíe el tráfico.

El firmware se escribe en la ranura que **no** está corriendo y el arranque se
cambia recién cuando `esp_ota_end()` valida el binario; una subida cortada no
rompe nada, sigue arrancando el viejo. El pie de la página dice qué ranura y
qué versión están corriendo.

Dos detalles que costaron un flasheo por cable descubrir en el nodo del refri y
que acá ya vienen resueltos: el handler necesita **8 kB de stack** (los 4 kB por
defecto se desbordan a mitad de la subida) y el buffer de recepción es
**estático**, no de pila.

## Registro histórico (CSV)

Lo que faltaba: **memoria larga**. En NVS hay 24 h de promedios por hora y 7
días de récords min/max, que alcanzan para mirar la pantalla pero no para
responder *"¿cómo se secó la maceta el mes pasado?"* ni para llevarse los datos
a una planilla.

Cada media hora en punto, `datalog.c` escribe una foto de todos los sensores
vivos en la partición `storage` (1 MB de SPIFFS que estaba declarada y sin
usar). Se descarga desde el panel:

```bash
curl -O http://<ip>/csv
```

```
fecha,sensor,valor
2026-07-24 15:30,pieza/temp,18.9
2026-07-24 15:30,pieza/suelo,74
```

- **Alineado al reloj**, no al arranque: las filas caen siempre en `:00` y
  `:30`, así las series de días distintos se comparan sin interpolar.
- **Los sensores mudos se saltan.** Repetir el último valor de un nodo caído
  dibujaría una línea recta que nunca existió; mejor un hueco.
- **No escribe nada hasta tener la hora.** Si no, el registro se llenaría de
  filas fechadas en 1970.
- **Rotación de dos archivos.** Cuando el actual pasa los 400 kB, el anterior se
  borra, el actual pasa a ser el anterior y se empieza uno nuevo. Nunca se copia
  ni se recorta nada: en 1 MB no hay lugar para duplicar el archivo mientras se
  lo trabaja. Quedan siempre entre uno y dos límites de historia — con ~13
  sensores, unas **3 a 6 semanas**.
- Si la partición no monta, el módulo se desactiva solo y el resto sigue igual.
  El montaje va en su propia tarea porque la primera vez SPIFFS **formatea**, y
  eso son varios segundos que si no retrasarían el arranque.

## Particiones

`partitions.csv`: dos ranuras de app de 3 MB (`ota_0`/`ota_1`) + 1 MB de
SPIFFS sobre 16 MB de flash. Las dos ranuras son lo que exige OTA. Quedan
~8.6 MB sin particionar.

La NVS conserva su offset y su tamaño a propósito: ahí viven las credenciales
Wi-Fi, los umbrales, los récords, el historial de alertas y los pasos. Moverla
equivaldría a borrarlos.

> ⚠️ Pasar de la tabla vieja (una sola app) a ésta **cambia el mapa de la
> flash**, así que ese flasheo puntual tiene que ser por cable. Los siguientes
> ya van por Wi-Fi.
