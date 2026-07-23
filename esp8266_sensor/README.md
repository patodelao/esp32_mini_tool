# esp8266_sensor

Nodo sensor basado en **ESP8266 (NodeMCU v3 / ESP-12E)**. Mide temperatura y
humedad del **aire** (DHT22) y humedad del **suelo** (módulo HW-103), vigila el
riego por su cuenta y publica todo por MQTT siguiendo las convenciones del
minitool.

Está pensado para **quedar instalado y no volver a tocarse**: se actualiza por
WiFi (OTA), se configura y se diagnostica a distancia, y se reinicia solo si
pierde la red.

> ⚠️ **Toolchain distinto al resto del repo.** Los demás proyectos
> (`minitool`, `opendoor_alarm`) son ESP-IDF (`idf.py`, target esp32s3). Este
> proyecto usa **PlatformIO + Arduino** para ESP8266 (xtensa-lx106). La
> extensión ESP-IDF de VS Code **no** compila ESP8266.

---

## 1. Conexionado (NodeMCU v3)

| Sensor | Pin sensor | Pin NodeMCU | Nota |
|---|---|---|---|
| DHT22 (aire) | VCC / GND / DATA | 3V3 / GND / **D2** | módulo 140C80 (pull-up incluido) |
| HW-103 (suelo) | VCC / GND / **A0** | **D5** / GND / **A0** | usar la salida analógica A0 del módulo, no D0 |

> ⚠️ **El VCC del HW-103 va a D5, no a 3V3.** La sonda resistiva se corroe si
> está siempre energizada (dura semanas en vez de meses), así que el firmware
> (`SUELO_VCC_PIN 14`) la enciende solo los 300 ms de cada medición. Si la dejás
> en 3V3, el nodo lo detecta y avisa *"sonda sin alimentación"* en vez de
> publicar humedad falsa. Para volver al cableado directo: `SUELO_VCC_PIN -1`.
>
> Si tu módulo trae LED de encendido puede pasar de los ~12 mA que da un GPIO;
> en ese caso desoldá el LED o usá un transistor pequeño.

> ⚠️ Alimentar todo a **3V3**, nunca a VU/VIN (5V): el GPIO y el ADC del ESP8266
> son de 3.3V y no toleran 5V.

Sella con termorretráctil la unión cable–sonda: se corroe justo en la línea de
la tierra húmeda.

### Calibración del suelo

El HW-103 es **resistivo**: seco da lectura ADC **alta**, en agua da **baja**.
Los dos extremos van en `main.cpp`:

```c
#define SUELO_SECO_RAW  1023   // raw con la sonda en tierra seca
#define SUELO_AGUA_RAW   520   // raw con la sonda en agua
```

> ⚠️ Medí `SECO_RAW` con la sonda en **tierra seca**, no al aire. Al aire la
> lectura se clava en el tope del ADC (1023) y eso cuesta dos cosas: los
> porcentajes quedan comprimidos (la tierra seca real nunca llega al tope), y no
> queda margen para distinguir "tierra seca" de "sonda al aire". Con `SECO_RAW`
> en 1023 la detección de sonda desconectada **se deshabilita sola, a propósito**,
> porque sería indistinguible de una medición normal.

Los dos umbrales de falla de sonda se **derivan** de esos números, nunca se
fijan a mano — un umbral fijo por debajo del punto de tierra seca hace que el
nodo confunda tierra seca con sonda rota y se quede mudo:

| Constante | Se calcula como | Qué detecta |
|---|---|---|
| `SUELO_RAW_DESCONECTADO` | `SECO_RAW + 8` (inalcanzable si `SECO_RAW` ≥ 1016) | sonda al aire, cable de señal suelto |
| `SUELO_RAW_MINIMO` | `AGUA_RAW / 4` | sonda sin alimentación o en corto |

En cualquiera de los dos casos el nodo **deja de publicar el %** (un valor al
revés de la realidad es peor que ninguno) y manda una alarma al bus, que repite
cada 30 min mientras dure.

---

## 2. Qué publica

Al broker `broker.hivemq.com`, con `<id>` = `NODE_ID` (por defecto `pieza`):

| Topic | Payload | Qué es | Retain |
|---|---|---|---|
| `labo/sensor/<id>/temp` | `"23.4"` (°C) | aire (DHT22), cada 10 s | sí |
| `labo/sensor/<id>/hum` | `"51.2"` (%) | aire (DHT22), cada 10 s | sí |
| `labo/sensor/<id>/suelo` | `"45"` (%) | suelo, al intervalo configurado | sí |
| `labo/sensor/<id>/suelo_raw` | `"604"` (ADC) | crudo, **solo en modo calibración** | no |
| `labo/sensor/<id>/rssi` | `"-58"` (dBm) | salud: señal WiFi, cada 60 s | sí |
| `labo/sensor/<id>/uptime` | `"143"` (min) | salud: minutos encendido | sí |
| `labo/sensor/<id>/heap` | `"28.4"` (kB) | salud: RAM libre | sí |
| `labo/nodo/<id>/status` | `online` / `offline` | estado (LWT) | sí |
| `labo/nodo/<id>/ip` | `"192.168.1.42"` | IP, para actualizarlo por OTA | sí |
| `labo/alerta/<id>` | JSON `{origen,nivel,msg}` | alertas de riego / sonda | no |

La telemetría de salud sale como **sensores normales**, así que el minitool ya
la grafica, le guarda récord del día y le aplica umbrales sin código extra:
avisa si el WiFi baja de −85 dBm o si la RAM libre cae bajo 8 kB (fuga).

El estado usa **Last-Will**: si el nodo cae sin avisar, el broker publica
`offline` por él. La IP aparece bajo el nombre del nodo en la tool **Nodos**.

## 3. Qué escucha

| Topic | Payload | Qué hace |
|---|---|---|
| `labo/config/<id>/suelo/umbral` | `"25"` (%) | riega bajo ese %. `0` = no vigilar |
| `labo/config/<id>/suelo/histeresis` | `"5"` (%) | margen para dar por recuperado |
| `labo/config/<id>/suelo/intervalo` | `"300"` (s) | cada cuánto mide el suelo (5 s – 1 h) |
| `labo/nodo/<id>/cmd` | `leer` | publica una lectura ya, sin esperar el ciclo |
| `labo/nodo/<id>/cmd` | `reset` | reinicia el nodo |
| `labo/nodo/<id>/cmd` | `cal on` / `cal off` | modo calibración |

Los tres `config/` los publica **retenidos** el editor de umbrales de la tool
Sensores (botón del engranaje), así que el nodo los recibe al conectar, incluso
después de un corte de luz. Hasta que lleguen usa `SUELO_UMBRAL_DEF` /
`SUELO_HISTER_DEF` / `SUELO_INTERVALO_DEF_S`.

Los comandos salen de la tool **Control** (tabla `s_cmds[]`).

---

## 4. Cómo alerta el riego

El nodo decide solo: **no depende de que el minitool esté encendido**. El
minitool solo le publica la configuración y muestra lo que el nodo manda al bus.

- **Confirmación:** necesita `SUELO_CONFIRMAR` (3) lecturas seguidas bajo el
  umbral antes de avisar. Un pico suelto del ADC no dispara nada.
  ⚠️ Cuenta **lecturas, no tiempo**: con intervalo de 5 min, una alerta tarda
  3 lecturas = 15 min en confirmarse.
- **Histéresis:** para volver a "normal" hay que superar `umbral + histéresis`,
  así el aviso no parpadea alrededor del límite.
- **Recordatorio:** mientras siga seco repite el aviso cada 30 min.
- **Mediana** de 9 lecturas del ADC en vez de promedio: descarta picos aislados.

El aire se publica siempre cada 10 s (`PUBLICAR_MS`); el suelo tiene su propio
ritmo porque la humedad de la tierra cambia en horas, no en segundos. Medir cada
varios minutos alcanza, energiza menos la sonda y ahorra tráfico.

### Modo calibración

`cal on` hace que el nodo mida cada segundo y publique el crudo del ADC en
`labo/sensor/<id>/suelo_raw`, que aparece como un sensor más ("Crudo Pieza") en
la tool Sensores. Sirve para **recalibrar y diagnosticar sin desmontar nada**:
mirás el número con la tierra seca y con la tierra recién regada, y esos son tus
`SUELO_SECO_RAW` / `SUELO_AGUA_RAW`, que después subís por OTA.

Se apaga solo a los 30 min para que no quede midiendo (y energizando la sonda)
por olvido. El crudo se publica **aunque la sonda esté marcada en falla**, que
es justo cuando más lo necesitás.

---

## 5. Compilar y subir por cable

Requisitos, una sola vez:

1. Extensión **PlatformIO IDE** en VS Code (convive con la de ESP-IDF).
2. Driver USB del NodeMCU: normalmente **CH340** (o CP2102).

```bash
pio run -e nodemcuv2 -t upload    # subir por USB
pio device monitor                # monitor serie (115200)
```

El **primer** flasheo tiene que ser por cable: el OTA no puede instalarse a sí
mismo.

## 6. Actualizar por WiFi (OTA)

Con el nodo ya instalado, hay un entorno dedicado en `platformio.ini` (el de
cable sigue disponible, por eso son dos entornos):

```bash
pio run -e pieza_ota -t upload
```

**Cómo funciona.** El nodo corre `ArduinoOTA.handle()` en cada vuelta del loop,
escuchando en el puerto UDP 8266 y anunciándose por mDNS como `pieza.local`.
PlatformIO lo invita a actualizarse, se autentican con un desafío MD5 (la clave
**no viaja** por la red), y entonces —esto es lo contraintuitivo— **el nodo se
conecta de vuelta a tu PC** para bajarse el firmware. Lo escribe en la mitad
libre de la flash, verifica el checksum y recién ahí reinicia. Si se corta a la
mitad, el firmware viejo queda intacto: **un OTA fallido no rompe nada**.

### ⚠️ El firewall de Windows bloquea el OTA

Como el nodo abre la conexión **hacia** tu PC, Windows la descarta si no hay una
regla. El síntoma es exactamente este:

```
Sending invitation to: pieza.local
Authenticating...OK
Waiting for device...
[ERROR]: No response from device
```

Que autentique y falle después **no es problema del nodo**: es tu firewall.

`platformio.ini` fija el puerto de retorno en 3232 (por defecto es aleatorio,
imposible de autorizar). Autorizalo una sola vez, en PowerShell **como
administrador**:

```powershell
New-NetFirewallRule -DisplayName "ESP OTA (espota)" -Direction Inbound -Protocol TCP -LocalPort 3232 -RemoteAddress 192.168.1.0/24 -Action Allow -Profile Any
```

Permite entrada **solo** al puerto 3232 y **solo** desde tu red local — no abre
nada a internet ni le da permisos generales a python. Ajustá `192.168.1.0/24` si
tu red usa otro rango. Para quitarla:
`Remove-NetFirewallRule -DisplayName "ESP OTA (espota)"`.

### Si `<id>.local` no resuelve

Pasa seguido en Windows. Usá la IP, que el nodo publica retenida y la tool
**Nodos** muestra bajo el nombre:

```
pieza    online
192.168.1.42
```

Cambiá `upload_port` en el entorno `[env:pieza_ota]`, o pasala en la línea:
`pio run -e pieza_ota -t upload --upload-port 192.168.1.42`.

Si el OTA falla incluso con la regla de firewall, el sospechoso siguiente son
los adaptadores virtuales (VPN, VirtualBox, WSL): forzá la ruta agregando
`--host_ip=<IP de tu PC>` a los `upload_flags`.

> 🔒 `OTA_PASSWORD` está en el fuente, así que queda compilada en el binario **y
> versionada en el repo**. Quien lea el código la sabe. El riesgo es acotado
> (hay que estar dentro de tu WiFi), pero no reutilices una clave de otro lado.

---

## 7. Replicar: agregar un nodo nuevo

Para un segundo nodo (digamos `cocina`), en **este** proyecto:

1. `NODE_ID "cocina"` en `main.cpp`. De ahí salen todos los topics.
2. Calibrá **esa** sonda en **esa** tierra: `SUELO_SECO_RAW` / `SUELO_AGUA_RAW`
   no son transferibles entre sondas ni entre macetas.
3. Un entorno OTA propio en `platformio.ini`, copiando `[env:pieza_ota]` con
   `upload_port = cocina.local`.
4. `ENABLE_DHT` / `ENABLE_SUELO` en 0 si ese nodo no lleva alguno de los dos.

Y en el **minitool**:

5. Agregá el nodo a la tabla `node_name` de `main/sensor_service.c` para que se
   muestre "Suelo Cocina" en vez de "suelo cocina". Sin esto funciona igual,
   solo se ve el id crudo.
6. Si querés comandarlo, sumá filas a `s_cmds[]` en `main/tools/tool_control.c`.

Los umbrales del nodo nuevo **no** hay que tocarlos en código: se editan desde
la tool Sensores y se guardan en NVS por id de sensor.

> ⚠️ **Límite:** el minitool guarda hasta `MAX_SENSORS` (16) sensores y otras
> tantas reglas. Un nodo completo ocupa hasta 7 topics (temp, hum, suelo, rssi,
> uptime, heap, suelo_raw), así que entran **dos** nodos completos más el refri.
> Para más, subí `MAX_SENSORS` en `sensor_service.c` y `MAX_RULES` en
> `sensor_alert.c` (tienen que quedar iguales).

Si **renombrás** un nodo existente, sus topics viejos quedan retenidos en el
broker y aparecen como sensores fantasma. Poné `LIMPIAR_TOPICS_VIEJOS 1` con
`ID_VIEJO` apuntando al nombre anterior, arrancá una vez, y volvelo a 0.

---

## 8. Referencia de configuración (`main.cpp`)

| Define | Qué es |
|---|---|
| `WIFI_SSID` / `WIFI_PASSWORD` | red 2.4 GHz (el ESP8266 no hace 5 GHz) |
| `NODE_ID` | nombre del nodo; raíz de todos sus topics |
| `OTA_PASSWORD` | clave de actualización por WiFi |
| `ENABLE_DHT` / `ENABLE_SUELO` | activar/desactivar cada sensor |
| `SUELO_VCC_PIN` | GPIO que alimenta la sonda (`14` = D5, `-1` = siempre alimentada) |
| `SUELO_SECO_RAW` / `SUELO_AGUA_RAW` | calibración; de acá se derivan los umbrales de falla |
| `SUELO_UMBRAL_DEF` / `SUELO_HISTER_DEF` | riego por defecto hasta que llegue la config del minitool |
| `SUELO_INTERVALO_DEF_S` | ritmo de medición por defecto |
| `SUELO_CONFIRMAR` | lecturas seguidas para confirmar seco / recuperado / falla |
| `CALIBRAR_SUELO` | estado inicial del modo calibración (se cambia por MQTT) |
| `LIMPIAR_TOPICS_VIEJOS` / `ID_VIEJO` | limpieza puntual tras renombrar el nodo |
| `PUBLICAR_MS` | ritmo del aire |
| `TELEMETRIA_MS` | ritmo de la telemetría de salud |
| `RESCATE_MS` | sin WiFi por más de esto, el nodo se reinicia solo |

En `platformio.ini`: `board = nodemcuv2` (vale para v2 y v3), `monitor_speed`,
`upload_speed`, y los entornos de subida.

## 9. Checklist de instalación

1. Cableá con el VCC del suelo en **D5** y flasheá **por cable**.
2. Calibrá en la maceta real (`cal on` → mirá "Crudo Pieza" con tierra seca y
   recién regada) y subí esos valores.
3. Cambiá `OTA_PASSWORD`.
4. **Probá una subida OTA antes de cerrar la caja** — incluida la regla de
   firewall. Si el OTA anda, todo lo demás ya se arregla a distancia.
5. Mirá el RSSI en la posición final (tool Sensores → `Wifi <nodo>`). Bajo
   −85 dBm vas a tener cortes.
6. Subí el intervalo del suelo a varios minutos desde el engranaje de la tool.

## Salida esperada por serie

```
=== ESP8266 sensor: aire (DHT22) + suelo (HW-103) + MQTT ===
Nodo: pieza
  aire  -> labo/sensor/pieza/temp | labo/sensor/pieza/hum (D2)
  suelo -> labo/sensor/pieza/suelo (A0)
  sonda de suelo alimentada por GPIO (anti-corrosion)
WiFi: conectado. IP 192.168.1.42  RSSI -58 dBm
OTA: activo (pio run -e pieza_ota -t upload)
MQTT: conectado
Aire  -> T: 23.4 C   HR: 51.2 %
Suelo -> 45 % (raw 604)
Salud -> RSSI -58 dBm   uptime 3 min   heap 44.1 kB
```
