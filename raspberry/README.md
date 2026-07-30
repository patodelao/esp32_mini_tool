# raspberry — servicios del home-lab en la Raspberry Pi

Lo que corre en la Pi (`192.168.1.100`, usuario `pi`), fuera de los ESP. Está
acá para poder redesplegarlo si se muere la SD.

## Broker MQTT

> **Cambio de topología (2026-07):** el broker **primario** del home-lab ya **no
> es la Pi**: es el nodo del refri (`opendoor_alarm`, `192.168.1.108`), que está
> siempre enchufado y corre un broker embebido (Mongoose). Los 4 nodos apuntan
> ahí. Motivo: la Pi se apaga o se reutiliza, y sin ella se cortaba la captura y
> la visualización de toda la red de sensores. El refri no duerme, así que la red
> ya no depende de la Pi. Ver `opendoor_alarm/main/mqtt_broker.c`.

**Mosquitto** (`sudo apt install mosquitto mosquitto-clients`), servicio
habilitado, escucha en `1883`. Config por defecto de Debian: `persistence true`
en `/var/lib/mosquitto/`. Ahora cumple un rol de **bridge/logging**: cuando la Pi
está prendida, espeja el tráfico del broker de opendoor (persistencia local,
insumos de la galería, `pi_monitor.py`). Cuando la Pi está apagada, opendoor
sigue siendo el broker y nadie se entera.

### Bridge hacia el broker de opendoor — `config/etc/mosquitto/conf.d/bridge.conf`

Hace que la Mosquitto de la Pi sea un espejo bidireccional de `labo/#` del broker
de opendoor. Como beneficio extra, **re-siembra los retenidos** de opendoor si el
refri se reinicia (OTA) mientras la Pi está viva — opendoor guarda los retenidos
en RAM y los pierde al rebootear.

```bash
sudo cp config/etc/mosquitto/conf.d/bridge.conf /etc/mosquitto/conf.d/
sudo systemctl restart mosquitto
mosquitto_sub -h localhost -t 'labo/#' -v    # debe ver el tráfico de la flota
```

`pi_monitor.py` y `cam_capture.py` siguen apuntando a `localhost`: sus mensajes
entran a la Mosquitto local y el bridge los lleva a opendoor.

## Seguridad

La Pi está en la LAN de casa (sin exposición a internet mientras el router no
haga port-forward). Endurecimiento aplicado:

**Firewall (`ufw`)** — Solo se aceptan conexiones desde `192.168.1.0/24`; el
resto se rechaza. Reglas en `manifests/ufw-status.txt`. Reconstruir:

```bash
sudo apt install -y ufw
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow from 192.168.1.0/24 to any port 22 proto tcp     comment 'SSH LAN'
sudo ufw allow from 192.168.1.0/24 to any port 1883 proto tcp   comment 'MQTT LAN'
sudo ufw allow from 192.168.1.0/24 to any port 139,445 proto tcp comment 'Samba tcp LAN'
sudo ufw allow from 192.168.1.0/24 to any port 137,138 proto udp comment 'Samba udp LAN'
sudo ufw allow from 192.168.1.0/24 to any port 5353 proto udp   comment 'mDNS LAN'
sudo ufw --force enable
sudo ufw logging off   # ver "Salud de la SD" abajo
```

**Salud de la SD** — En una Pi los logs desgastan la tarjeta. Dos cosas dejan
`/var/log` tranquilo sin montar `tmpfs` (que agrega riesgo en el arranque):
- **journald** ya está en modo **volátil** (escribe a RAM `/run`, no a la SD) —
  es el default cuando no existe `/var/log/journal`.
- **`ufw logging off`**: el logging de ufw estaba llenando `kern.log` (llegó a
  51 MB) con cada paquete de broadcast/multicast bloqueado de la LAN. Apagarlo
  cortó de raíz la escritura pesada. Con esto `/var/log` bajó de 58 MB a ~7 MB.

**SSH solo por clave** — `PasswordAuthentication no` y `PermitRootLogin no` en
`config/etc/ssh/sshd_config`. El acceso es con la clave `id_ed25519_homelab`
(ver [[raspberry-broker]] en memoria). Tras editar: `sudo sshd -t && sudo
systemctl reload ssh`.

**Samba** — Los recursos (`roms`, `bios`, `configs`, `splashscreens`) siguen
como `guest`, pero ahora solo alcanzables desde la LAN por el firewall. Si en el
futuro se quiere exigir contraseña: quitar `guest ok = yes` de cada share, poner
`valid users = pi` y crear la clave con `sudo smbpasswd -a pi`.

### Migración del broker a MQTT con auth

Hoy el broker es anónimo. El firmware de los 4 nodos ya está **preparado** para
mandar usuario/clave (definiendo `MQTT_USER`/`MQTT_PASS` en el `secrets` de cada
nodo); mientras no se definan, siguen conectando anónimos. Para exigir auth sin
cortar la flota:

1. **Crear el usuario en la Pi** (elegí una clave; se pide dos veces):
   ```bash
   sudo mosquitto_passwd -c /etc/mosquitto/passwd fleet
   ```
2. **Activar el archivo de auth** dejando `allow_anonymous true` (los nodos
   viejos siguen andando):
   ```bash
   sudo cp config/etc/mosquitto/conf.d/auth.conf /etc/mosquitto/conf.d/
   sudo systemctl restart mosquitto
   ```
3. **Reflashear los nodos uno por uno** con las credenciales:
   - ESP-IDF (`minitool`, `esp32cam`, `opendoor_alarm`): descomentar
     `MQTT_USER`/`MQTT_PASS` en su `secrets.h` y flashear (OTA o cable).
   - ESP8266 (`esp8266_sensor` = *pieza*): descomentar las líneas
     `-DMQTT_USER/-DMQTT_PASS` en `platformio.ini` y agregar `mqtt_user`/
     `mqtt_pass` a `secrets.ini`, luego flashear.
   El usuario/clave debe ser el mismo `fleet` del paso 1 en los 4.
4. **Cerrar el anónimo**: cuando los 4 ya conectan con credenciales, editar
   `/etc/mosquitto/conf.d/auth.conf` a `allow_anonymous false` y
   `sudo systemctl restart mosquitto`. Verificar en la tool Nodos del minitool
   que los 4 siguen `online`.

## Agente de captura de la cámara — `cam_capture.py`

Baja `http://<cam>/foto.jpg` cada X minutos y guarda las fotos con fecha en
`~/homelab/fotos/`. Se **configura desde el minitool** (tool **Camara**), que
publica retenido:

| Topic | Valor |
|---|---|
| `labo/config/cam/captura/activo` | `1`/`0` |
| `labo/config/cam/captura/intervalo` | minutos |

La IP del cam la toma sola de `labo/nodo/cam/ip`. Como feedback publica
`labo/sensor/cam/fotos` (total guardadas), que aparece en la tool Sensores.
Guarda como máximo `MAX_FILES` (3000) fotos, borrando las más viejas.

### Instalación

```bash
sudo apt install -y python3-paho-mqtt
mkdir -p ~/homelab/fotos
cp cam_capture.py ~/homelab/cam_capture.py
sudo cp homelab-cam.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now homelab-cam
```

### Ver las fotos guardadas — galería web (`gallery/`)

`gallery/gallery.py` sirve las fotos de `~/homelab/fotos/` en una página web,
más nuevas primero y paginada. Solo usa la biblioteca estándar de Python (nada
que instalar) y corre como servicio systemd. El firewall limita el puerto a la
LAN, igual que el resto.

Abrir desde cualquier equipo de casa:

```
http://192.168.1.100:8088/     (o http://retropie.local:8088/)
```

Instalación:

```bash
cp gallery/gallery.py ~/homelab/gallery.py
sudo cp gallery/homelab-gallery.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now homelab-gallery
sudo ufw allow from 192.168.1.0/24 to any port 8088 proto tcp comment 'Galeria cam LAN'
```

## Auto-monitoreo de la Pi — `monitor/pi_monitor.py`

La Pi se publica a sí misma a MQTT como **un nodo más** (id `pi`, que el minitool
muestra como *RetroPi*), siguiendo las convenciones de la flota:

| Topic | Contenido |
|---|---|
| `labo/nodo/pi/status` | `online`/`offline` (retenido, con last-will) |
| `labo/nodo/pi/ip` | IP en la LAN |
| `labo/sensor/pi/temp` | temperatura CPU, °C |
| `labo/sensor/pi/uptime` | encendido, min |
| `labo/sensor/pi/rssi` | señal WiFi, dBm |
| `labo/sensor/pi/cpu` | uso de CPU, % |
| `labo/sensor/pi/disco` | uso de `/`, % |

Así aparece sola en las tools **Nodos** y **Sensores** del minitool. Requirió
subir `MAX_SENSORS` de 16 a 24 en el minitool (`sensor_service.c`) — con la Pi la
flota pasó de 16, que ya se desbordaba — y agregar el label `pi`→`RetroPi` y las
unidades de `cpu`/`disco`. Solo stdlib + paho-mqtt.

```bash
cp monitor/pi_monitor.py ~/homelab/pi_monitor.py
sudo cp monitor/homelab-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now homelab-monitor
```

## Estructura de esta carpeta

```
raspberry/
├── cam_capture.py          agente de captura de la cámara
├── homelab-cam.service     unit systemd del agente
├── gallery/
│   ├── gallery.py          galería web de las fotos (stdlib, puerto 8088)
│   └── homelab-gallery.service
├── monitor/
│   ├── pi_monitor.py       la Pi se auto-publica a MQTT (nodo 'pi' = RetroPi)
│   └── homelab-monitor.service
├── scripts/
│   ├── backup-homelab.sh   respaldo de lo irrecuperable (ver abajo)
│   └── pi-snapshot.sh      refresca config/ y manifests/ desde el PC
├── config/                 copia versionada de la config REAL de la Pi
│   └── etc/{mosquitto,samba,ssh}/...
└── manifests/              inventario para reconstruir igual
    ├── apt-manual.txt          paquetes instalados a mano
    ├── systemd-enabled.txt     servicios habilitados
    ├── ufw-status.txt          reglas del firewall
    ├── crontab.txt             tareas cron (pi + root)
    └── retropie.txt            versión RetroPie + conteo de ROMs
```

`config/` y `manifests/` son una **foto** del estado de la Pi para poder
reconstruirla. Si cambias algo en la Pi, refresca esta foto con
`scripts/pi-snapshot.sh` (ver abajo) y commitea.

## Backup de lo irrecuperable — `scripts/backup-homelab.sh`

La Pi es una tarjeta SD: se puede corromper. Este script respalda lo que **no
se vuelve a bajar**: partidas guardadas, configs de emuladores/mandos,
gamelists y carátulas de EmulationStation, los datos del home-lab y los
mensajes retenidos del broker. **No** respalda las ROMs (pesadas y
re-descargables).

```bash
# en la Pi:
bash ~/homelab/backup-homelab.sh          # crea ~/homelab/backups/homelab-backup-FECHA.tar.gz
# bajarlo al PC:
scp homelab:'~/homelab/backups/homelab-backup-*.tar.gz' .
```

Conserva los últimos 7 backups. Para automatizarlo semanalmente, agregar a
`crontab -e` de `pi`:

```
0 4 * * 0  /usr/bin/bash /home/pi/homelab/backup-homelab.sh >> /home/pi/homelab/backup.log 2>&1
```

## Reconstruir la Pi de cero

Si la SD muere y hay que empezar de nuevo:

1. Flashear **Raspberry Pi OS** (o RetroPie) y habilitar SSH.
2. Copiar la clave pública para acceso sin contraseña (`ssh-copy-id` o el
   `authorized_keys`), y dejar el `~/.ssh/config` del PC apuntando a la Pi.
3. Reinstalar servicios base:
   ```bash
   sudo apt update
   sudo apt install -y mosquitto mosquitto-clients python3-paho-mqtt samba
   ```
4. Restaurar las configs de `config/etc/...` (revisar antes de pisar las del
   sistema) y reinstalar el agente de cámara (sección de arriba).
5. Restaurar el último backup:
   ```bash
   sudo tar -xzf homelab-backup-FECHA.tar.gz -C /
   ```
6. Comparar paquetes instalados contra `manifests/apt-manual.txt` y RetroPie
   contra `manifests/retropie.txt`.
