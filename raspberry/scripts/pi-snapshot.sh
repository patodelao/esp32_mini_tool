#!/usr/bin/env bash
#
# pi-snapshot.sh — refresca la "foto" versionada de la Pi (config/ + manifests/).
#
# Se corre DESDE EL PC (no en la Pi), dentro del repo. Usa el alias SSH `homelab`
# definido en ~/.ssh/config. No toca la Pi: solo lee y baja archivos.
#
#   bash raspberry/scripts/pi-snapshot.sh
#
set -euo pipefail

SSH_HOST="${SSH_HOST:-homelab}"
# Ubicarse en raspberry/ (un nivel arriba de scripts/).
cd "$(dirname "$0")/.."

echo "Refrescando config/ y manifests/ desde ${SSH_HOST} ..."

mkdir -p config/etc/mosquitto/conf.d config/etc/samba config/etc/ssh manifests

# --- Config real (tal cual) ---
scp -q "${SSH_HOST}:/etc/mosquitto/mosquitto.conf" config/etc/mosquitto/mosquitto.conf
scp -q "${SSH_HOST}:/etc/samba/smb.conf"           config/etc/samba/smb.conf
scp -q "${SSH_HOST}:/etc/ssh/sshd_config"          config/etc/ssh/sshd_config

# --- Manifiestos (inventario) ---
ssh -o BatchMode=yes "$SSH_HOST" 'apt-mark showmanual' > manifests/apt-manual.txt
ssh -o BatchMode=yes "$SSH_HOST" 'systemctl list-unit-files --state=enabled --no-legend --no-pager | awk "{print \$1}"' > manifests/systemd-enabled.txt
ssh -o BatchMode=yes "$SSH_HOST" 'crontab -l 2>/dev/null; echo "### root:"; sudo crontab -l 2>/dev/null' > manifests/crontab.txt
ssh -o BatchMode=yes "$SSH_HOST" 'cat /opt/retropie/VERSION 2>/dev/null; echo; echo "roms:"; for d in /home/pi/RetroPie/roms/*/; do n=$(ls -1 "$d" 2>/dev/null | wc -l); [ "$n" -gt 0 ] && echo "$n $(basename "$d")"; done | sort -rn' > manifests/retropie.txt

echo "Listo. Revisa los cambios con: git status && git diff"
