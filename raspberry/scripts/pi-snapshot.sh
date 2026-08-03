#!/usr/bin/env bash
#
# pi-snapshot.sh — refresca la "foto" versionada de la Pi (config/ + manifests/).
#
# Se corre DESDE EL PC (no en la Pi), dentro del repo. No toca la Pi: solo lee.
#
#   bash raspberry/scripts/pi-snapshot.sh
#
# Por defecto usa el alias SSH `homelab` (~/.ssh/config). Si te conectás por
# contraseña o con otro host, pasalo así:
#   SSH_HOST=pi@192.168.1.100 bash raspberry/scripts/pi-snapshot.sh
#
# Abre UNA conexión SSH compartida (te pide la clave una sola vez), saltea los
# archivos que no existan y no pisa un manifest si el comando falla.

set -uo pipefail

SSH_HOST="${SSH_HOST:-homelab}"
cd "$(dirname "$0")/.."

echo "Refrescando config/ y manifests/ desde ${SSH_HOST} ..."

# --- Conexión SSH compartida: una sola autenticacion para todo el script ---
CTRL="$(mktemp -u)"
trap 'ssh -o ControlPath="$CTRL" -O exit "$SSH_HOST" 2>/dev/null || true' EXIT
ssh -o ControlMaster=yes -o ControlPath="$CTRL" -o ControlPersist=120 -fN "$SSH_HOST" \
  || { echo "No pude conectar a $SSH_HOST"; exit 1; }
S=(-o ControlPath="$CTRL")

mkdir -p config/etc/mosquitto/conf.d config/etc/samba config/etc/ssh manifests \
         config/boot config/etc/systemd/system/getty@tty1.service.d \
         config/opt/retropie/configs/all config/home/pi

# Baja un archivo si existe; si no, lo saltea sin abortar.
grab() { scp -q "${S[@]}" "${SSH_HOST}:$1" "$2" 2>/dev/null && echo "  ok  $1" || echo "  --  $1 (no existe, salteado)"; }

# Guarda un manifest SOLO si el comando devolvio algo (no pisa con vacio).
save() { local out; out=$(ssh "${S[@]}" "$SSH_HOST" "$2" 2>/dev/null) && [ -n "$out" ] \
         && printf '%s\n' "$out" > "manifests/$1" && echo "  ok  manifests/$1" \
         || echo "  --  manifests/$1 (sin datos, no tocado)"; }

# --- Config real (tal cual esta en la Pi) ---
grab /etc/mosquitto/mosquitto.conf config/etc/mosquitto/mosquitto.conf
grab /etc/samba/smb.conf           config/etc/samba/smb.conf
grab /etc/ssh/sshd_config          config/etc/ssh/sshd_config
# Arranque + autostart de RetroPie
grab /boot/firmware/config.txt                                  config/boot/config.txt
grab /etc/systemd/system/getty@tty1.service.d/autologin.conf    config/etc/systemd/system/getty@tty1.service.d/autologin.conf
grab /opt/retropie/configs/all/autostart.sh                     config/opt/retropie/configs/all/autostart.sh
grab /home/pi/.bash_profile                                     config/home/pi/.bash_profile

# --- Manifiestos (inventario para reconstruir igual) ---
save apt-manual.txt      'apt-mark showmanual'
save systemd-enabled.txt 'systemctl list-unit-files --state=enabled --no-legend --no-pager | awk "{print \$1}"'
save crontab.txt         'crontab -l 2>/dev/null; echo "### root:"; sudo -n crontab -l 2>/dev/null'
save retropie.txt        'cat /opt/retropie/VERSION 2>/dev/null; echo; echo "roms:"; for d in /home/pi/RetroPie/roms/*/; do n=$(ls -1 "$d" 2>/dev/null | wc -l); [ "$n" -gt 0 ] && echo "$n $(basename "$d")"; done | sort -rn'

echo "Listo. Revisa los cambios con: git status && git diff"
