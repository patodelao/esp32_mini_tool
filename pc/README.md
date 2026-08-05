# pc/ — presencia del PC de escritorio en el home-lab

`pc_presence.py` hace que el PC **avise por MQTT si esta encendido o apagado**,
para que Home Assistant (y la automatizacion "PC apagado -> apagar TV") reaccione
en segundos.

## Por que no por ping

Tras "Apagar", Windows (Inicio rapido + Wake-on-LAN) deja la placa de red
respondiendo pings varios minutos, asi que un `binary_sensor` de tipo *ping*
tardaba ~13 min en marcar el PC como apagado. Con un **Last Will** de MQTT el
broker publica `offline` en cuanto el PC se va de la red (~20 s en un corte duro,
instantaneo si el apagado es limpio). Bonus: no depende de la IP del PC (adios al
lio de la IP dinamica `.90`).

## Instalar (en el PC, PowerShell)

```powershell
py -m pip install paho-mqtt
```

1. Copia `pc_presence.py` a una ruta estable, p. ej. `C:\Users\<tu-usuario>\homelab\`.
2. Verifica que el broker de la Pi es alcanzable:
   `Test-NetConnection 192.168.1.100 -Port 1883`  (debe dar `TcpTestSucceeded : True`)
3. Probalo una vez con consola: `py C:\...\homelab\pc_presence.py`
   -> debe imprimir "conectado al broker" y en HA aparece `binary_sensor.pc_escritorio`.
4. Arranque automatico: `Win+R` -> `shell:startup` -> crea un acceso directo a
   `pyw "C:\...\homelab\pc_presence.py"` (con **pyw**, sin ventana).

## Topics

- `labo/nodo/pc/estado` (retenido): `online` / `offline`.
- `homeassistant/binary_sensor/pc_escritorio/config` (retenido): discovery para HA.

## Automatizacion en HA

Dispara con `binary_sensor.pc_escritorio` a `off`. Ver la automatizacion
"PC apagado -> apagar TV" (manda `labo/nodo/pieza/tv` = `off`, reintenta x3
mientras `light.tira_led` no este `unavailable`).
