#!/usr/bin/env bash
#
# backup-homelab.sh — respaldo de lo IRRECUPERABLE de la Pi (RetroPi).
#
# NO respalda las ROMs (son pesadas y se vuelven a bajar). Sí respalda lo que
# no se puede recuperar: partidas guardadas, configs de emuladores y mandos,
# gamelists/favoritos de EmulationStation, los datos del home-lab (scripts +
# mensajes retenidos del broker) y las configs de sistema versionables.
#
# Genera un tar.gz con fecha en ~/homelab/backups/ y conserva los ultimos N.
# Pensado para correr en la Pi (por cron o a mano). Para bajarlo al PC:
#   scp homelab:'~/homelab/backups/<archivo>' .
#
set -euo pipefail

DEST="${HOME}/homelab/backups"
KEEP=7                       # cuantos backups conservar
STAMP="$(date +%Y%m%d-%H%M)"
OUT="${DEST}/homelab-backup-${STAMP}.tar.gz"

mkdir -p "$DEST"

# Lista de saves dentro de las ROMs (RetroPie guarda al lado del juego).
SAVES_LIST="$(mktemp)"
trap 'rm -f "$SAVES_LIST"' EXIT
find "${HOME}/RetroPie/roms" \
     \( -iname '*.srm' -o -iname '*.state*' -o -iname '*.sav' \
        -o -iname '*.rtc' -o -iname '*.mcr' -o -iname '*.ss*' \) \
     -type f 2>/dev/null > "$SAVES_LIST" || true

echo "Respaldando en ${OUT} ..."

# --ignore-failed-read: no aborta si falta alguna ruta opcional.
# sudo: /etc y /var/lib/mosquitto no siempre son legibles por 'pi'.
sudo tar --ignore-failed-read -czf "$OUT" \
    --exclude="${HOME}/homelab/fotos" \
    --exclude="${HOME}/homelab/backups" \
    --exclude='.git' \
    --exclude='/opt/retropie/configs/all/retroarch/shaders' \
    --exclude='/opt/retropie/configs/all/retroarch/assets' \
    -T "$SAVES_LIST" \
    "/opt/retropie/configs" \
    "${HOME}/.emulationstation/gamelists" \
    "${HOME}/.emulationstation/es_settings.cfg" \
    "${HOME}/homelab" \
    "/var/lib/mosquitto" \
    "/etc/mosquitto" \
    "/etc/samba/smb.conf" \
    "/etc/ssh/sshd_config" \
    2>/dev/null || true

sudo chown "$(id -un):$(id -gn)" "$OUT"

# Rotacion: conservar solo los ultimos $KEEP.
ls -1t "${DEST}"/homelab-backup-*.tar.gz 2>/dev/null | tail -n +$((KEEP+1)) | xargs -r rm -f

echo "Listo: $(du -h "$OUT" | cut -f1)  ${OUT}"
echo "Backups actuales:"
ls -1sh "${DEST}"/homelab-backup-*.tar.gz
