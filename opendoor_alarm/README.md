# opendoor_alarm

Proyecto ESP-IDF para alarma de puerta de refrigerador **y broker MQTT del
home-lab**.

El nodo esta **siempre enchufado y despierto**: vigila la puerta de forma
continua, publica telemetria cada minuto y deja el servidor de actualizacion
escuchando todo el tiempo. (Antes dormia en deep sleep y despertaba por GPIO
al abrirse la puerta; eso ahorraba una bateria que no existe y hacia
imposible actualizarlo o pedirle algo mientras dormia.)

## Broker MQTT del home-lab (2026-07)

Justo por estar siempre encendido, este nodo **hospeda el broker MQTT** de toda
la flota (antes vivia en la Raspberry Pi, que se apaga o se reutiliza y dejaba a
la red de sensores sin broker). Corre un broker embebido sobre **Mongoose**
(`components/mongoose/` + `main/mqtt_broker.c`) escuchando en `0.0.0.0:1883`; el
resto de los nodos (pieza, cam, minitool) se conectan a `192.168.1.108`.

El propio refri publica su telemetria/puerta y escucha sus comandos **sin abrir
un cliente TCP a si mismo**: usa la API in-process del broker
(`mqtt_broker_local_publish` / `mqtt_broker_on_local`). Ya no usa el cliente
esp-mqtt. Detalles y limitaciones (retenidos en RAM, sin last-will, anonimo) en
la cabecera de `main/mqtt_broker.h`.

## Hardware

- Sensor magnético: GPIO 10 (entrada con pull-up, activo en nivel alto).
- Buzzer pasivo: GPIO 5 (LEDC, tono 2.5 kHz).
- LED de alarma:
  - Intenta usar WS2812 integrado en GPIO48 (ESP32-S3) vía RMT.
  - Fallback por pines RGB: R=GPIO2, G=GPIO3, B=GPIO4.

## Flujo

1. Inicializa GPIO, buzzer y LED.
2. Conecta a Wi-Fi y **levanta el broker MQTT local** (Mongoose, puerto 1883);
   publica su presencia y arranca el OTA.
3. Si la puerta está abierta al arrancar, publica `ABIERTO` en `labo/nodo/refri/puerta`.
4. Mientras la puerta siga abierta:
   - desde los 20 s: LED alternando rojo/verde.
   - desde los 30 s: buzzer intermitente sincronizado con el parpadeo.
   - cada 5 s: publica heartbeat `ABIERTO` por MQTT.
5. Al detectar cierre, **calla el buzzer y apaga el LED en el acto** y recién
   entonces confirma que el cierre es estable, en silencio, por 3 s. El imán
   rebota y por eso hay que confirmar, pero antes eso pasaba con el buzzer
   todavía sonando: cerrabas la puerta y seguía pitando un par de segundos.
6. Si se confirma, publica `CERRADO` (QoS 1). Si se reabre antes, la vuelta
   siguiente del bucle vuelve a encender la alarma sola.
7. Vuelve a vigilar. No duerme: sigue publicando telemetria cada minuto y
   atendiendo el servidor de actualizacion.

## MQTT — integración con el minitool

Además del topic original de la puerta, el nodo publica según las convenciones
del minitool, de modo que aparece solo en sus tools (Nodos, Sensores) y en el
bus de alertas multicanal, sin cambios del lado del minitool.

| Topic | Payload | Uso | Retain |
|---|---|---|---|
| `labo/nodo/refri/puerta` | `ABIERTO` / `CERRADO` | Dashboard del minitool | no |
| `labo/nodo/refri/status` | `online` / `offline` | Tool **Nodos** (estado) | sí |
| `labo/alerta/refri` | `{"origen","nivel","msg"}` | Bus de **alertas multicanal** | no |
| `labo/sensor/refri/rssi` | dBm | Tool **Sensores** (señal Wi-Fi) | sí |
| `labo/sensor/refri/uptime` | minutos | Tool **Sensores** (salud) | sí |
| `labo/sensor/refri/heap` | kB libres | Tool **Sensores** (salud) | sí |
| `labo/nodo/refri/ip` | `192.168.1.x` | Tool **Nodos** (para llegarle) | sí |
| `labo/sensor/refri/abierta_seg` | segundos | Tool **Sensores** (duración de la apertura) | sí |

> El estado de la puerta va bajo `labo/nodo/` y no bajo `labo/sensor/` porque
> el payload es texto (`ABIERTO`/`CERRADO`), no un número: los sensores del
> minitool asumen números para graficar y aplicar umbrales.
>
> Antes se publicaba en `proyectos/casa/refri/puerta`. El minitool sigue
> escuchando ese topic viejo por compatibilidad; ya se puede quitar de
> `alert_service.c`, porque el nodo está actualizado.

### Estado online/offline

- `online` mientras el nodo está encendido y conectado.
- Si cae de golpe (corte de energía, pérdida de Wi-Fi), el **LWT** del broker
  publica `offline` por él.

Con la telemetría cada minuto, el silencio prolongado de un nodo que dice estar
online es señal real de problema, y el minitool lo avisa. Mientras dormía en
deep sleep esto no se podía distinguir de la operación normal.

### Niveles de la alerta multicanal (`labo/alerta/refri`)

- `aviso` — al abrirse la puerta.
- `alarma` — cuando escala (buzzer, apertura prolongada).
- `ok` — al confirmarse el cierre.

## Configuración rápida

Credenciales en `main/secrets.h` (ignorado por git; copiar de
`secrets.h.example`): `WIFI_SSID`, `WIFI_PASSWORD` y `OTA_PASSWORD`.

En `main/main.c`: los pines, si tu placa usa otro mapeo, y los tiempos de
escalada de la alarma (`LED_START_DELAY_MS`, `BUZZER_START_DELAY_MS`).

## Compilar y cargar

```bash
idf.py set-target esp32s3
idf.py fullclean
idf.py build
idf.py -p <PUERTO> flash monitor
```

---

## Actualizar por Wi-Fi (OTA)

El nodo levanta un servidor HTTP mínimo y recibe el firmware por POST. No hace
falta servidor externo ni nube: se sube desde la misma PC donde compilás.

```bash
curl -X POST --data-binary @build/opendoor_alarm.bin "http://<ip>/update?key=<OTA_PASSWORD>"
```

Probado de punta a punta: 950 kB en **7.8 s**, con el nodo pasando de la ranura
`ota_0` a la `ota_1`. La página de estado dice en cuál está corriendo y con qué
versión (el hash del commit), así que siempre se puede confirmar qué firmware
tiene puesto.

La IP se ve en la tool **Nodos** del minitool (el nodo la publica retenida), o
entrando con el navegador a `http://<ip>/`, que muestra una página con la
ranura activa y el tiempo encendido.

**Por qué distinto al ESP8266.** El nodo de la pieza usa ArduinoOTA, que es de
Arduino y no existe en ESP-IDF. Acá el modelo se invierte: el que abre la
conexión es tu PC, no el nodo. Eso tiene una ventaja concreta — **no depende
del firewall de Windows**, que fue justo lo que costó hacer andar el otro.

### Requisitos que esto impuso

- **Tabla de particiones con dos ranuras** (`partitions.csv`): el firmware
  nuevo se escribe en la que no está corriendo y el arranque se cambia recién
  al terminar y verificar. Una subida cortada no rompe nada: sigue arrancando
  el viejo.
- **Flash de 4 MB** (`sdkconfig.defaults`). Estaba configurado en 2 MB, donde
  dos ranuras no entran. 4 MB es el mínimo de cualquier módulo ESP32-S3.
- El **primer flasheo con esta tabla tiene que ser por cable**: cambia el mapa
  de la flash, así que no puede aplicarse a sí mismo por OTA.

> 🔒 La clave viaja en la URL sobre HTTP plano. Sirve para que nadie de tu red
> flashee el refri por accidente, no contra alguien que esté espiando el
> tráfico. Para eso haría falta HTTPS con certificados.

## Comandos (tool Control)

| Topic | Payload | Qué hace |
|---|---|---|
| `labo/nodo/refri/cmd` | `reset` | reinicia el nodo |
| `labo/nodo/refri/cmd` | `leer` | republica telemetría e IP ya |
