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
