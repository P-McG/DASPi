#!/bin/bash
set -euo pipefail

sudo -v

readonly IMG="../golden/aperturecomputemodule.img"
readonly SYSROOT_DIR="../sysroot/rootfs"
readonly MNT="../work/sysroot-mnt"

LOOP_DEV=""

log() {
    echo "[sysroot] $*"
}

cleanup() {
    local exit_code=$?
    set +e

    if mountpoint -q "$MNT"; then
        sudo umount "$MNT"
    fi

    if [[ -n "$LOOP_DEV" ]]; then
        sudo losetup -d "$LOOP_DEV"
    fi

    exit "$exit_code"
}
trap cleanup EXIT

fix_alt_managed_symlink() {
    local subdir="$1"      # e.g. blas
    local libname="$2"     # e.g. libblas.so
    local top_link="$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/${libname}.3"
    local real_dir="$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/$subdir"

    if [[ ! -d "$real_dir" ]]; then
        log "warning: missing directory $real_dir"
        return 0
    fi

    local real_file
    real_file="$(find "$real_dir" -maxdepth 1 -type f -name "${libname}.3*" | sort -V | tail -n 1)"

    if [[ -z "${real_file:-}" ]]; then
        log "warning: no ${libname}.3* found in $real_dir"
        return 0
    fi

    sudo rm -f "$top_link"
    sudo ln -s "${subdir}/$(basename "$real_file")" "$top_link"

    log "fixed $(basename "$top_link") -> ${subdir}/$(basename "$real_file")"
}

fix_plain_so_symlink() {
    local libdir="$SYSROOT_DIR/usr/lib/aarch64-linux-gnu"
    local stem="$1"

    local real_file
    real_file="$(find "$libdir" -maxdepth 1 -type f -name "${stem}.so.*" | sort -V | tail -n 1)"

    if [[ -z "${real_file:-}" ]]; then
        log "warning: no ${stem}.so.* found in $libdir"
        return 0
    fi

    sudo rm -f "$libdir/${stem}.so"
    sudo ln -s "$(basename "$real_file")" "$libdir/${stem}.so"
    log "fixed ${stem}.so -> $(basename "$real_file")"
}

mkdir -p "$MNT"
mkdir -p "$(dirname "$SYSROOT_DIR")"

log "cleaning old sysroot"
sudo rm -rf "$SYSROOT_DIR"
mkdir -p "$SYSROOT_DIR"

if [[ ! -f "$IMG" ]]; then
    log "missing image: $IMG"
    exit 1
fi

log "mapping image"
LOOP_DEV="$(sudo losetup --show -fP "$IMG")"
ROOT_DEV="${LOOP_DEV}p2"

log "mounting rootfs"
sudo mount "$ROOT_DEV" "$MNT"

log "copying rootfs"
sudo rsync -aHAX --delete \
    --numeric-ids \
    "$MNT"/ "$SYSROOT_DIR"/

log "fixing alternative-managed BLAS/LAPACK symlinks for cross-linking"
fix_alt_managed_symlink "blas" "libblas.so"
fix_alt_managed_symlink "lapack" "liblapack.so"

fix_plain_so_symlink "libcamera"
fix_plain_so_symlink "libcamera-base"

log "fixing ownership"
sudo chown -R "$USER:$USER" "$SYSROOT_DIR"

log "done -> $SYSROOT_DIR"
log "verifying ownership"
ls -ld "$SYSROOT_DIR"

log "verifying BLAS/LAPACK links"
ls -l "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/libblas.so.3" || true
ls -l "$SYSROOT_DIR/usr/lib/aarch64-linux-gnu/liblapack.so.3" || true

read -p "Press Enter to exit..."
