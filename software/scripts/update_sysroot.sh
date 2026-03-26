#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="${HOME}/DASPi/sysroots"
REMOTE_USER="${REMOTE_USER:-$USER}"

# host:name
TARGETS=(
  "aperturecomputemodule000:aperturecomputemodule"
  "computemodule000:computemodule"
)

LIB_EXCLUDES=(
  "/cups/***"
)

USR_EXCLUDES=(
  "/lib/cups/***"
  "/share/polkit-1/rules.d/***"
  "/share/doc/***"
  "/share/man/***"
  "/share/locale/***"
)

log() {
  printf '[INFO] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*" >&2
}

ensure_dest_dirs() {
  local dest="$1"
  mkdir -p \
    "${dest}/lib" \
    "${dest}/usr" \
    "${dest}/etc"
}

build_exclude_args() {
  local -n arr="$1"
  local pattern
  for pattern in "${arr[@]}"; do
    printf -- '--exclude=%s\n' "$pattern"
  done
}

rsync_dir() {
  local host="$1"
  local remote_dir="$2"
  local local_dir="$3"
  shift 3
  local extra_args=("$@")

  rsync -aH --delete \
    "${extra_args[@]}" \
    "${REMOTE_USER}@${host}:${remote_dir}" \
    "${local_dir}"
}

rsync_optional_dir() {
  local label="$1"
  shift
  if ! "$@"; then
    warn "${label} had non-fatal rsync issues; continuing"
  fi
}

sync_ld_so_conf() {
  local host="$1"
  local dest_etc="$2"

  mkdir -p "${dest_etc}"
  rsync -a \
    "${REMOTE_USER}@${host}:/etc/ld.so.conf" \
    "${dest_etc}/" || true

  mkdir -p "${dest_etc}/ld.so.conf.d"
  rsync -a \
    "${REMOTE_USER}@${host}:/etc/ld.so.conf.d/" \
    "${dest_etc}/ld.so.conf.d/" || true
}

sync_lib64_if_present() {
  local host="$1"
  local dest="$2"

  if ssh "${REMOTE_USER}@${host}" 'test -d /lib64'; then
    mkdir -p "${dest}/lib64"
    rsync_optional_dir "lib64 sync for ${host}" \
      rsync_dir "${host}" "/lib64/" "${dest}/lib64/"
  fi
}

rewrite_absolute_symlinks() {
  local dest="$1"

  log "Rewriting absolute symlinks in ${dest}"

  python3 - "$dest" <<'PY'
import os
import sys
from pathlib import Path

sysroot = Path(sys.argv[1]).resolve()

for path in sysroot.rglob("*"):
    try:
        if not path.is_symlink():
            continue
        target = os.readlink(path)
        if not target.startswith("/"):
            continue

        resolved = sysroot / target.lstrip("/")
        if not resolved.exists():
            print(f"[WARN] unresolved absolute symlink: {path} -> {target}")
            continue

        rel = os.path.relpath(resolved, start=path.parent)
        path.unlink()
        path.symlink_to(rel)
        print(f"[FIX] {path} -> {rel}")
    except OSError as e:
        print(f"[WARN] failed to process {path}: {e}")
PY
}

repair_blas_lapack_links() {
  local dest="$1"
  local libdir="${dest}/usr/lib/aarch64-linux-gnu"

  mkdir -p "${libdir}"

  if [[ -f "${libdir}/blas/libblas.so.3.11.0" ]]; then
    ln -sfn "blas/libblas.so.3.11.0" "${libdir}/libblas.so.3"
    ln -sfn "libblas.so.3" "${libdir}/libblas.so"
    log "Repaired BLAS symlinks in ${libdir}"
  fi

  if [[ -f "${libdir}/lapack/liblapack.so.3.11.0" ]]; then
    ln -sfn "lapack/liblapack.so.3.11.0" "${libdir}/liblapack.so.3"
    ln -sfn "liblapack.so.3" "${libdir}/liblapack.so"
    log "Repaired LAPACK symlinks in ${libdir}"
  fi
}

update_target() {
  local host="$1"
  local name="$2"
  local dest="${BASE_DIR}/${name}"

  log "Updating ${name} from ${host}"
  ensure_dest_dirs "${dest}"

  mapfile -t lib_exclude_args < <(build_exclude_args LIB_EXCLUDES)
  mapfile -t usr_exclude_args < <(build_exclude_args USR_EXCLUDES)

  rsync_optional_dir "lib sync for ${name}" \
    rsync_dir "${host}" "/lib/" "${dest}/lib/" "${lib_exclude_args[@]}"

  rsync_optional_dir "usr sync for ${name}" \
    rsync_dir "${host}" "/usr/" "${dest}/usr/" "${usr_exclude_args[@]}"

  sync_lib64_if_present "${host}" "${dest}"
  sync_ld_so_conf "${host}" "${dest}/etc"

  rewrite_absolute_symlinks "${dest}"
  repair_blas_lapack_links "${dest}"

  log "Finished ${name}"
}

main() {
  mkdir -p "${BASE_DIR}"

  local entry host name
  for entry in "${TARGETS[@]}"; do
    host="${entry%%:*}"
    name="${entry#*:}"
    update_target "${host}" "${name}"
  done

  log "Sysroot update complete."
}

main "$@"
