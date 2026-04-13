#!/bin/bash
set -euo pipefail

export SYSROOT="$HOME/DASPi/images/aperturecomputemodule/sysroot/rootfs"
export PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
export PKG_CONFIG_LIBDIR="$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig"
export PKG_CONFIG_PATH=""

echo "[env] SYSROOT=$SYSROOT"

pkg-config --modversion libcamera
pkg-config --modversion libevent
pkg-config --cflags libevent
pkg-config --libs libevent

rm -rf ~/DASPi/software/build/aperturecomputemodule

meson setup ~/DASPi/software/build/aperturecomputemodule \
  ~/DASPi/software/src \
  --cross-file ~/DASPi/software/src/aperturecomputemodule/cross-aperturecomputemodule.txt \
  -Dbuild_aperturecomputemodule=true \
  -Dbuild_computemodule=false

meson compile -C ~/DASPi/software/build/aperturecomputemodule
