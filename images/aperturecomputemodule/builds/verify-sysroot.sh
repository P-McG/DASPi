#!/bin/bash
set -euo pipefail

SYSROOT="../sysroot/rootfs"

echo "[verify] checking basic structure"
test -d "$SYSROOT/usr"
test -d "$SYSROOT/usr/include"
test -d "$SYSROOT/usr/lib"

echo "[verify] checking libcamera"
find "$SYSROOT/usr" -path '*libcamera.h' || true
find "$SYSROOT/usr" -path '*pkgconfig/libcamera.pc' || true
find "$SYSROOT/usr/lib" -name 'libcamera*.so*' || true

echo "[verify] checking opencv"
find "$SYSROOT/usr" -path '*opencv4*' || true

echo "[verify] done"

read -p "Press Enter to exit..."
