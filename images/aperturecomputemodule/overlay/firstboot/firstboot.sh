#!/bin/bash
set -Eeuo pipefail

LOG_FILE="/var/log/firstboot.log"
DONE_FILE="/boot/firmware/firstboot.done"
MARKER_FILE="/etc/firstboot-ran"
LOCK_FILE="/run/firstboot.lock"

exec > >(tee -a "$LOG_FILE") 2>&1

on_error() {
    local exit_code="$1"
    local line_no="$2"
    echo "[firstboot] ERROR: command failed at line ${line_no} with exit code ${exit_code}"
    echo "[firstboot] leaving without writing done marker"
    exit "$exit_code"
}
trap 'on_error $? $LINENO' ERR

echo "[firstboot] started $(date -Is)"
id

if [[ -e "$DONE_FILE" || -e "$MARKER_FILE" ]]; then
    echo "[firstboot] already completed previously; exiting"
    exit 0
fi

if [[ -e "$LOCK_FILE" ]]; then
    echo "[firstboot] lock file exists ($LOCK_FILE); another run may be in progress"
    exit 1
fi
touch "$LOCK_FILE"

cleanup() {
    rm -f "$LOCK_FILE"
}
trap cleanup EXIT

echo "firstboot started at $(date -Is)" > "$MARKER_FILE"

detect_wired_iface() {
    local iface
    for iface in end0 eth0; do
        if ip link show "$iface" >/dev/null 2>&1; then
            echo "$iface"
            return 0
        fi
    done
    ip -o link show | awk -F': ' '{print $2}' | grep -E '^(en|eth)' | head -n1
}

wait_for_systemd_unit() {
    local unit="$1"
    local attempts="${2:-10}"
    local delay="${3:-2}"
    local i
    for ((i=1; i<=attempts; ++i)); do
        if systemctl list-unit-files "$unit" >/dev/null 2>&1; then
            return 0
        fi
        echo "[firstboot] waiting for unit ${unit} (${i}/${attempts})"
        sleep "$delay"
    done
    return 1
}

ensure_service_active() {
    local unit="$1"
    local attempts="${2:-5}"
    local delay="${3:-2}"
    local i
    for ((i=1; i<=attempts; ++i)); do
        echo "[firstboot] enabling ${unit} (${i}/${attempts})"
        systemctl enable "$unit"
        echo "[firstboot] starting ${unit} (${i}/${attempts})"
        systemctl start "$unit" || true
        if systemctl is-active --quiet "$unit"; then
            echo "[firstboot] ${unit} is active"
            return 0
        fi
        echo "[firstboot] ${unit} not active yet; retrying after ${delay}s"
        systemctl status "$unit" --no-pager -l || true
        sleep "$delay"
    done
    echo "[firstboot] ${unit} failed to become active"
    systemctl status "$unit" --no-pager -l || true
    return 1
}

IFACE="$(detect_wired_iface || true)"
echo "[firstboot] iface=${IFACE:-<none>}"

MAC=""
if [[ -n "${IFACE:-}" && -r "/sys/class/net/${IFACE}/address" ]]; then
    MAC="$(tr '[:upper:]' '[:lower:]' < "/sys/class/net/${IFACE}/address")"
fi
echo "[firstboot] mac=${MAC:-<none>}"

HOSTNAME_VALUE="unknown-pi"
case "${MAC}" in
    88:a2:9e:ba:6b:2c)
        HOSTNAME_VALUE="aperturecomputemodule000"
        ;;
esac
echo "[firstboot] hostname=${HOSTNAME_VALUE}"

echo "$HOSTNAME_VALUE" > /etc/hostname

if grep -q '^127\.0\.1\.1' /etc/hosts; then
    sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t${HOSTNAME_VALUE}/" /etc/hosts
else
    echo -e "127.0.1.1\t${HOSTNAME_VALUE}" >> /etc/hosts
fi

hostnamectl set-hostname "$HOSTNAME_VALUE" || true

echo "[firstboot] blocking wifi"
rfkill block wifi || true

echo "[firstboot] generating ssh host keys"
ssh-keygen -A

echo "[firstboot] waiting for ssh unit"
wait_for_systemd_unit ssh.service 10 2

echo "[firstboot] enabling and starting ssh"
ensure_service_active ssh.service 5 2

echo "[firstboot] setting timezone"
timedatectl set-timezone America/Toronto || true

echo "[firstboot] forcing time sync"
systemctl restart systemd-timesyncd || true
sleep 5

mkdir -p /boot/firmware
touch "$DONE_FILE"

echo "[firstboot] completed $(date -Is)"
exit 0
