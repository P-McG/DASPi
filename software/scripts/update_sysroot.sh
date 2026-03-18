#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="${HOME}/DASPi/sysroots"
REMOTE_USER="${REMOTE_USER:-$USER}"

# host:name
TARGETS=(
  "aperturecomputemodule000:aperturecomputemodule"
  "computemodule000:computemodule"
)

# Paths relative to each rsync source root.
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
    "${dest}/opt" \
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

  rsync -a --delete \
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

  rsync -a \
    "${REMOTE_USER}@${host}:/etc/ld.so.conf*" \
    "${dest_etc}/" || true
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

  rsync_optional_dir "opt sync for ${name}" \
    rsync_dir "${host}" "/opt/" "${dest}/opt/"

  sync_ld_so_conf "${host}" "${dest}/etc"

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
