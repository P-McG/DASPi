#!/usr/bin/env bash
set -euo pipefail

# ----------------------------------------
# DASPi aperturecomputemodule build script
# ----------------------------------------

DASPI_ROOT="${DASPI_ROOT:-$HOME/DASPi}"

PXE_USER="${PXE_USER:-daspi}"
PXE_HOST="${PXE_HOST:-10.0.2.2}"
PXE_PATH="${PXE_PATH:-/srv/nfs/default/}"

SRC_DIR="$DASPI_ROOT/software/src"
BUILD_DIR="$DASPI_ROOT/software/build/aperturecomputemodule"

BASE_CROSS_FILE="$SRC_DIR/common/cross-aarch64-base.ini"

export DASPI_SYSROOT="${DASPI_SYSROOT:-$DASPI_ROOT/images/aperturecomputemodule/sysroot/rootfs/default}"

SYNC_SYSROOT="${SYNC_SYSROOT:-1}"
CLEAN_BUILD_DIR="${CLEAN_BUILD_DIR:-1}"
DEPLOY="${DEPLOY:-1}"
DEPLOY_PATH="${DEPLOY_PATH:-/srv/nfs/default/opt/daspi}"
TARGET_BIN="$BUILD_DIR/aperturecomputemodule/aperturecomputemodule"

log() {
  echo "[build-aperturecomputemodule] $*"
}

fail() {
  echo "[build-aperturecomputemodule] ERROR: $*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || fail "Missing file: $1"
}

require_dir() {
  [[ -d "$1" ]] || fail "Missing directory: $1"
}

sync_sysroot() {
  log "Syncing sysroot from ${PXE_USER}@${PXE_HOST}:${PXE_PATH}"
  mkdir -p "$DASPI_SYSROOT"

  rsync -aHAX --delete --numeric-ids \
    --info=progress2 \
    "${PXE_USER}@${PXE_HOST}:${PXE_PATH}" \
    "$DASPI_SYSROOT" \
    --exclude='/dev' \
    --exclude='/proc' \
    --exclude='/sys' \
    --exclude='/tmp' \
    --exclude='/run' \
    --exclude='/mnt' \
    --exclude='/media' \
    --exclude='/lost+found' \
    --exclude='/home' \
    --exclude='/root' \
    --exclude='/var/run' \
    --exclude='/var/tmp' \
    --exclude='/var/log' \
    --exclude='/var/cache' \
    --exclude='/var/lib/apt/lists/partial' \
    --exclude='/var/lib/bluetooth' \
    --exclude='/var/lib/logrotate'
}

repair_sysroot_symlinks() {
  log "Repairing sysroot BLAS/LAPACK symlinks"

  local libdir="$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu"
  cd "$libdir"

  rm -f libopenblas.so libopenblas.so.0 libblas.so libblas.so.3 liblapack.so liblapack.so.3

  ln -s openblas-pthread/libopenblas.so libopenblas.so
  ln -s openblas-pthread/libopenblas.so.0 libopenblas.so.0
  ln -s libopenblas.so libblas.so
  ln -s libopenblas.so.0 libblas.so.3
  ln -s liblapack.so.3 liblapack.so
  ln -s openblas-pthread/libopenblas.so liblapack.so.3
}

generate_cross_overlay() {
  GENERATED_CROSS_FILE="$BUILD_DIR/cross-aperturecomputemodule.generated.ini"

  log "Generating cross overlay: $GENERATED_CROSS_FILE"
  mkdir -p "$BUILD_DIR"

  cat > "$GENERATED_CROSS_FILE" <<EOF
[properties]
sys_root = '$DASPI_SYSROOT'
pkg_config_libdir = [
  '$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig',
  '$DASPI_SYSROOT/usr/lib/pkgconfig',
  '$DASPI_SYSROOT/usr/share/pkgconfig',
]
pkg_config_sysroot_dir = '$DASPI_SYSROOT'

[built-in options]
c_args = [
  '--sysroot=$DASPI_SYSROOT',
]
cpp_args = [
  '--sysroot=$DASPI_SYSROOT',
]
c_link_args = [
  '--sysroot=$DASPI_SYSROOT',
  '-L$DASPI_SYSROOT/lib/aarch64-linux-gnu',
  '-L$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu',
  '-Wl,-rpath-link,$DASPI_SYSROOT/lib/aarch64-linux-gnu',
  '-Wl,-rpath-link,$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu',
]
cpp_link_args = [
  '--sysroot=$DASPI_SYSROOT',
  '-L$DASPI_SYSROOT/lib/aarch64-linux-gnu',
  '-L$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu',
  '-Wl,-rpath-link,$DASPI_SYSROOT/lib/aarch64-linux-gnu',
  '-Wl,-rpath-link,$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu',
]

[cmake]
CMAKE_SYSTEM_NAME = 'Linux'
CMAKE_SYSTEM_PROCESSOR = 'aarch64'
CMAKE_SYSROOT = '$DASPI_SYSROOT'
CMAKE_FIND_ROOT_PATH = '$DASPI_SYSROOT'
CMAKE_FIND_ROOT_PATH_MODE_PROGRAM = 'NEVER'
CMAKE_FIND_ROOT_PATH_MODE_LIBRARY = 'ONLY'
CMAKE_FIND_ROOT_PATH_MODE_INCLUDE = 'ONLY'
CMAKE_FIND_ROOT_PATH_MODE_PACKAGE = 'ONLY'
EOF
}

sanity_check_sysroot() {
  log "Checking sysroot"

  require_dir "$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu"
  require_file "$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig/libcamera.pc"
  require_file "$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig/opencv4.pc"
}

sanity_check_pkgconfig() {
  log "Checking pkg-config"

  export PKG_CONFIG_SYSROOT_DIR="$DASPI_SYSROOT"
  export PKG_CONFIG_LIBDIR="$DASPI_SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$DASPI_SYSROOT/usr/lib/pkgconfig:$DASPI_SYSROOT/usr/share/pkgconfig"
  export PKG_CONFIG_PATH=""

  pkg-config --modversion libcamera
  pkg-config --modversion libevent
  pkg-config --modversion opencv4
}

configure_build() {
  log "Configuring Meson"

  if [[ "$CLEAN_BUILD_DIR" == "1" ]]; then
    rm -rf "$BUILD_DIR"
  fi

  generate_cross_overlay

  meson setup "$BUILD_DIR" "$SRC_DIR" \
    --cross-file "$BASE_CROSS_FILE" \
    --cross-file "$GENERATED_CROSS_FILE" \
    -Dbuild_aperturecomputemodule=true \
    -Dbuild_computemodule=false
}

compile_build() {
  log "Compiling"
  meson compile -C "$BUILD_DIR"
}

deploy_binary() {
  log "Deploying binary to ${PXE_USER}@${PXE_HOST}:${DEPLOY_PATH}"

  [[ -f "$TARGET_BIN" ]] || fail "Binary not found: $TARGET_BIN"

  ssh "${PXE_USER}@${PXE_HOST}" "mkdir -p '$DEPLOY_PATH'"

  rsync -av \
    "$TARGET_BIN" \
    "${PXE_USER}@${PXE_HOST}:$DEPLOY_PATH/"

  ssh "${PXE_USER}@${PXE_HOST}" "ls -l '$DEPLOY_PATH/aperturecomputemodule'"
}

main() {
  log "DASPI_ROOT=$DASPI_ROOT"
  log "DASPI_SYSROOT=$DASPI_SYSROOT"
  log "PXE source=${PXE_USER}@${PXE_HOST}:${PXE_PATH}"

  require_dir "$SRC_DIR"
  require_file "$BASE_CROSS_FILE"

  if [[ "$SYNC_SYSROOT" == "1" ]]; then
    sync_sysroot
  else
    log "Skipping sysroot sync"
  fi

  repair_sysroot_symlinks
  sanity_check_sysroot
  sanity_check_pkgconfig
  configure_build
  compile_build

  if [[ "$DEPLOY" == "1" ]]; then
    deploy_binary
  else
    log "Skipping deploy"
  fi

  log "Build completed successfully"
}

main "$@"
