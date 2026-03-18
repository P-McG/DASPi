#!/usr/bin/env bash
set -Eeuo pipefail

: "${DRY_RUN:=0}"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

run_cmd() {
    if [[ "$DRY_RUN" == "1" ]]; then
        printf '[DRY-RUN] '
        printf '%q ' "$@"
        printf '\n'
    else
        "$@"
    fi
}

remote_run() {
    local host="$1"
    shift
    if [[ "$DRY_RUN" == "1" ]]; then
        printf '[DRY-RUN] ssh -o BatchMode=yes -o ConnectTimeout=5 %q %q\n' "$host" "$*"
    else
        ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" "$@"
    fi
}

ensure_remote_dir() {
    local host="$1"
    local dir="$2"
    remote_run "$host" "mkdir -p '$dir'"
}

remove_remote_dir() {
    local host="$1"
    local dir="$2"
    remote_run "$host" "rm -rf '$dir'"
}

copy_file() {
    local src="$1"
    local host="$2"
    local dst="$3"

    remote_run "$host" "mkdir -p '$(dirname "$dst")'"

    if [[ "$DRY_RUN" == "1" ]]; then
        printf '[DRY-RUN] scp %q %q:%q\n' "$src" "$host" "$dst"
    else
        scp "$src" "$host:$dst"
    fi
}

copy_tree() {
    local src="$1"
    local host="$2"
    local dst="$3"

    remote_run "$host" "mkdir -p '$dst'"

    if [[ "$DRY_RUN" == "1" ]]; then
        printf '[DRY-RUN] scp -r %q/* %q:%q/\n' "$src" "$host" "$dst"
    else
        scp -r "$src"/* "$host:$dst"/
    fi
}

stop_tmux_session() {
    local host="$1"
    local session_name="$2"
    remote_run "$host" "tmux kill-session -t '$session_name' 2>/dev/null || true"
}

start_tmux_session() {
    local host="$1"
    local session_name="$2"
    local remote_cmd="$3"

    remote_run "$host" "
        tmux kill-session -t '$session_name' 2>/dev/null || true
        tmux new-session -d -s '$session_name' '$remote_cmd'
    "
}
