#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BASE_IMG="$SCRIPT_DIR/../base/raspios-lite.img"
BASE_IMG_XZ="$SCRIPT_DIR/../base/2025-12-04-raspios-trixie-arm64-lite.img.xz"
WORK_DIR="$SCRIPT_DIR/../work"
OUTPUT_IMG="$SCRIPT_DIR/../golden/aperturecomputemodule.img"

BOOT_MNT="$WORK_DIR/bootfs"
ROOT_MNT="$WORK_DIR/rootfs"

OVERLAY_SRC="$SCRIPT_DIR/../overlay"

APERTURE_BIN=../../../software/build/aperturecomputemodule/aperturecomputemodule/aperturecomputemodule

FIRSTBOOT_SCRIPT_SRC="$OVERLAY_SRC/firstboot/firstboot.sh"
FIRSTBOOT_SERVICE_SRC="$OVERLAY_SRC/firstboot/firstboot.service"

APERTURE_SERVICE_SRC="$OVERLAY_SRC/services/aperturecomputemodule.service"
APERTURE_DEFAULTS_SRC="$OVERLAY_SRC/services/aperturecomputemodule.default"
APERTURE_LAUNCHER_SRC="$OVERLAY_SRC/services/launch-aperturecomputemodule.sh"

PI_USER="daspi"
PI_PASS="daspi"
HOSTNAME_VALUE="aperturecomputemodule000"

LOOP_DEV=""

cleanup() {
    set +e
    sync

    if mountpoint -q "$ROOT_MNT"; then
        sudo umount "$ROOT_MNT" || sudo umount -l "$ROOT_MNT"
    fi

    if mountpoint -q "$BOOT_MNT"; then
        sudo umount "$BOOT_MNT" || sudo umount -l "$BOOT_MNT"
    fi

    if [[ -n "${LOOP_DEV:-}" ]]; then
        sudo losetup -d "$LOOP_DEV" 2>/dev/null || true
    fi
}

trap cleanup EXIT

require_file() {
    local path="$1"
    local label="$2"
    if [[ ! -f "$path" ]]; then
        echo "[build] missing ${label}: $path"
        exit 1
    fi
}

echo "[build] preparing workspace"
mkdir -p "$WORK_DIR"
mkdir -p "$(dirname "$OUTPUT_IMG")"

echo "[build] resetting workspace mounts"
sudo umount "$BOOT_MNT" 2>/dev/null || true
sudo umount "$ROOT_MNT" 2>/dev/null || true
rm -rf "$BOOT_MNT" "$ROOT_MNT"
mkdir -p "$BOOT_MNT" "$ROOT_MNT"

if [[ ! -f "$BASE_IMG" ]]; then
    echo "[build] extracting base image"
    xz -dk "$BASE_IMG_XZ"
fi

require_file "$BASE_IMG" "base image"
require_file "$APERTURE_BIN" "aperturecomputemodule binary"
require_file "$FIRSTBOOT_SCRIPT_SRC" "firstboot script"
require_file "$FIRSTBOOT_SERVICE_SRC" "firstboot service"
require_file "$APERTURE_SERVICE_SRC" "aperturecomputemodule service"
require_file "$APERTURE_DEFAULTS_SRC" "aperturecomputemodule defaults"
require_file "$APERTURE_LAUNCHER_SRC" "aperturecomputemodule launcher"

echo "[build] copying base image"
rm -f "$OUTPUT_IMG"
cp "$BASE_IMG" "$OUTPUT_IMG"

echo "[build] mapping partitions"
LOOP_DEV="$(sudo losetup --show -fP "$OUTPUT_IMG")"
BOOT_DEV="${LOOP_DEV}p1"
ROOT_DEV="${LOOP_DEV}p2"

echo "[build] mounting partitions"
sudo mount "$BOOT_DEV" "$BOOT_MNT"
sudo mount "$ROOT_DEV" "$ROOT_MNT"

sudo touch "$ROOT_MNT/etc/ssh/sshd_config"

sudo ln -sf /usr/lib/systemd/system/ssh.service \
  "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/ssh.service"

ls "$ROOT_MNT/usr/lib/systemd/system/ssh.service"

echo "[build] enabling ssh service"
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf /usr/lib/systemd/system/ssh.service \
    "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/ssh.service"

echo "[build] allowing ssh password auth"
sudo sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication yes/' \
    "$ROOT_MNT/etc/ssh/sshd_config"
sudo sed -i 's/^#\?UsePAM .*/UsePAM yes/' \
    "$ROOT_MNT/etc/ssh/sshd_config"

echo "[build] forcing fresh host keys on first boot"
sudo rm -f "$ROOT_MNT/etc/ssh/ssh_host_"*

echo "[build] resetting cloud-init state"
sudo rm -rf "$ROOT_MNT/var/lib/cloud/"*
sudo truncate -s 0 "$ROOT_MNT/etc/machine-id"
sudo rm -f "$ROOT_MNT/var/lib/dbus/machine-id"

echo "[build] writing boot config"
sudo touch "$BOOT_MNT/ssh"
sudo rm -f "$BOOT_MNT/userconf" "$BOOT_MNT/userconf.txt"
sudo rm -f "$BOOT_MNT/user-data" "$BOOT_MNT/meta-data" "$BOOT_MNT/network-config"

cat <<EOF | sudo tee "$BOOT_MNT/user-data" >/dev/null
#cloud-config
hostname: ${HOSTNAME_VALUE}
users:
  - name: ${PI_USER}
    gecos: DASPi User
    groups: [adm, sudo, video, plugdev, render]
    shell: /bin/bash
    lock_passwd: false
    plain_text_passwd: ${PI_PASS}
ssh_pwauth: true
disable_root: true
EOF

cat <<EOF | sudo tee "$BOOT_MNT/meta-data" >/dev/null
instance-id: ${HOSTNAME_VALUE}-instance
local-hostname: ${HOSTNAME_VALUE}
EOF

echo "[build] verifying user-data"
sudo head -n 20 "$BOOT_MNT/user-data"
echo "[build] verifying meta-data"
sudo cat "$BOOT_MNT/meta-data"

echo "[build] installing firstboot artifacts"
sudo install -D -m 755 "$FIRSTBOOT_SCRIPT_SRC" \
    "$ROOT_MNT/usr/local/sbin/firstboot.sh"
sudo install -D -m 644 "$FIRSTBOOT_SERVICE_SRC" \
    "$ROOT_MNT/etc/systemd/system/firstboot.service"

echo "[build] enabling firstboot service"
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf ../firstboot.service \
    "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/firstboot.service"
sudo rm -f "$ROOT_MNT/boot/firmware/firstboot.done"

echo "[build] installing aperturecomputemodule runtime"
sudo install -D -m 755 "$APERTURE_BIN" \
    "$ROOT_MNT/opt/daspi/aperturecomputemodule"
sudo install -D -m 755 "$APERTURE_LAUNCHER_SRC" \
    "$ROOT_MNT/opt/daspi/bin/launch-aperturecomputemodule.sh"
sudo install -D -m 644 "$APERTURE_SERVICE_SRC" \
    "$ROOT_MNT/etc/systemd/system/aperturecomputemodule.service"
sudo install -D -m 644 "$APERTURE_DEFAULTS_SRC" \
    "$ROOT_MNT/etc/default/aperturecomputemodule"

echo "[build] creating runtime directories"
sudo install -d -m 755 "$ROOT_MNT/var/log/aperturecomputemodule"

echo "[build] applying static overlay payload"
if [[ -d "$OVERLAY_SRC/files" ]]; then
    sudo cp -a "$OVERLAY_SRC/files/." "$ROOT_MNT/"
fi

echo "[build] enabling aperturecomputemodule service"
sudo ln -sf ../aperturecomputemodule.service \
    "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/aperturecomputemodule.service"

echo "[build] removing old startup paths"
sudo rm -f "$ROOT_MNT/etc/rc.local"
sudo rm -f "$ROOT_MNT/etc/systemd/system/rc-local.service"
sudo rm -f "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/rc-local.service"
sudo rm -f "$ROOT_MNT/etc/systemd/system/daspi.service"
sudo rm -f "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/daspi.service"
sudo rm -f "$ROOT_MNT/usr/local/bin/daspi"

echo "[build] syncing"
sync

echo "[build] unmounting"
cleanup
trap - EXIT

echo "[build] done -> $OUTPUT_IMG"

read -p "Press Enter to exit..."
