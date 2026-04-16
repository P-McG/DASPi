#!/usr/bin/env bash
set -euo pipefail

PXE_USER="${PXE_USER:-user}"
PXE_HOST="${PXE_HOST:-10.0.2.2}"
PXE_PATH="${PXE_PATH:-/srv/nfs/}"
DASPI_ROOT="${DASPI_ROOT:-$HOME/DASPi}"
SYSROOT="${SYSROOT:-$DASPI_ROOT/images/aperturecomputemodule/sysroot/rootfs}"

echo "[sync] PXE source : ${PXE_USER}@${PXE_HOST}:${PXE_PATH}"
echo "[sync] Sysroot    : ${SYSROOT}"

mkdir -p "${SYSROOT}"

rsync -aHAX --delete --numeric-ids \
  --info=progress2 \
  --rsync-path="sudo rsync" \
  "${PXE_USER}@${PXE_HOST}:${PXE_PATH}" \
  "${SYSROOT}" \
  --exclude='/dev/*' \
  --exclude='/proc/*' \
  --exclude='/sys/*' \
  --exclude='/tmp/*' \
  --exclude='/run/*' \
  --exclude='/mnt/*' \
  --exclude='/media/*' \
  --exclude='/lost+found' \
  --exclude='/var/tmp/*' \
  --exclude='/var/run/*'

echo "[sync] done"
