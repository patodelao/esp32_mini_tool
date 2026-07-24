# opendoor_alarm

Proyecto ESP-IDF para alarma de puerta de refrigerador.

El nodo esta **siempre enchufado y despierto**: vigila la puerta de forma
continua, publica telemetria cada minuto y deja el servidor de actualizacion
escuchando todo el tiempo. (Antes dormia en deep sleep y despertaba por GPIO
al abrirse la puerta; eso ahorraba una bateria que no existe y hacia
imposible actualizarlo o pedirle algo mientras dormia.)

## Hardware

- Sensor magnético: GPIO 10 (entrada con pull-up, activo en nivel alto).
- Buzzer pasivo: GPIO 5 (LEDC, tono 2.5 kHz).
- LED de alarma:
  - Intenta usar WS2812 integrado en GPIO48 (ESP32-S3) vía RMT.
  - Fallback por pines RGB: R=GPIO2, G=GPIO3, B=GPIO4.

## Flujo

1. Inicializa GPIO, buzzer y LED.
2. Conecta a Wi-Fi y a MQTT (`mqtt://broker.hivemq.com`).
3. Si la puerta está abierta al arrancar, publica `ABIERTO` en `labo/nodo/refri/puerta` (QoS 1).
4. Mientras la puerta siga abierta:
   - desde los 20 s: LED alternando rojo/verde.
   - desde los 30 s: buzzer intermitente sincronizado con el parpadeo.
   - cada 5 s: publica heartbeat `ABIERTO` por MQTT.
5. Al detectar cierre, apaga salidas y confirma cierre estable por 3 s.
6. Si se confirma, publica `CERRADO` (QoS 1). Si se reabre antes, vuelve a alarma.
7. Configura wakeup por GPIO 10 y entra en deep sleep.

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
> escuchando ese topic viejo por compatibilidad (este nodo no tiene OTA, así
> que hasta que lo flashees por cable sigue usándolo); se puede quitar de
> `alert_service.c` una vez actualizado.

### Estado online/offline (importante)

El nodo vive en **deep sleep** y solo despierta al abrirse la puerta. Por eso:

- `online` = despierto (puerta abierta / alarma activa).
- `offline` = durmiendo (puerta cerrada), publicado de forma limpia antes de dormir.
- Si el nodo cae de golpe (corte de energía, pérdida de Wi-Fi), el **LWT** del
  broker publica `offline` por él.

No es un "heartbeat de salud" continuo (eso gastaría batería): refleja
despierto/durmiendo. Para health real habría que despertar por temporizador
cada cierto tiempo y publicar; se puede agregar si se necesita.

### Niveles de la alerta multicanal (`labo/alerta/refri`)

- `aviso` — al abrirse la puerta.
- `alarma` — cuando escala (buzzer, apertura prolongada).
- `ok` — al confirmarse el cierre.

## Configuración rápida

Edita en `main/main.c`:

- `WIFI_SSID`
- `WIFI_PASSWORD`
- Pines si tu placa usa otro mapeo.

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
