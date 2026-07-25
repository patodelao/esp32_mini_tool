# raspberry — servicios del home-lab en la Raspberry Pi

Lo que corre en la Pi (`192.168.1.100`, usuario `pi`), fuera de los ESP. Está
acá para poder redesplegarlo si se muere la SD.

## Broker MQTT

**Mosquitto** (`sudo apt install mosquitto mosquitto-clients`), servicio
habilitado, escucha en `1883`. Config por defecto de Debian: `persistence true`
en `/var/lib/mosquitto/`, así que los mensajes retenidos sobreviven a un
reinicio de la Pi. Es el broker al que apuntan los 4 nodos.

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

### Ver las fotos guardadas

Por ahora se acceden entrando a la Pi (`~/homelab/fotos/`). Pendiente: una
galería web para verlas desde el navegador sin SSH.

## Estructura de esta carpeta

```
raspberry/
├── cam_capture.py          agente de captura de la cámara
├── homelab-cam.service     unit systemd del agente
├── scripts/
│   └── backup-homelab.sh   respaldo de lo irrecuperable (ver abajo)
├── config/                 copia versionada de la config REAL de la Pi
│   └── etc/{mosquitto,samba,ssh}/...
└── manifests/              inventario para reconstruir igual
    ├── apt-manual.txt          paquetes instalados a mano
    ├── systemd-enabled.txt     servicios habilitados
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
