#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOFTWARE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_deploy.sh"

require_cmd ssh
require_cmd scp
require_cmd tmux
require_cmd mkdir
require_cmd cp
require_cmd chmod

increment_subnet() {
    local ip="$1"
    IFS='.' read -r o1 o2 o3 o4 <<< "$ip"

    ((o3++))
    if ((o3 > 255)); then
        echo "Error: Subnet (3rd octet) overflow" >&2
        return 1
    fi

    echo "$o1.$o2.$o3.$o4"
}

HOSTS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        *)
            HOSTS+=("$1")
            shift
            ;;
    esac
done

PORT="${PORT:-5000}"
CLIENT="${CLIENT:-10.0.2.2}"
VERBOSE="${VERBOSE:---verbose}"

if [[ "${#HOSTS[@]}" -eq 0 ]]; then
    HOSTS=("aperturecomputemodule000" "aperturecomputemodule001")
fi

LOCAL_BUILD_EXE="${LOCAL_BUILD_EXE:-$(find "$HOME/DASPi/software/build/aperturecomputemodule" -type f -executable -name aperturecomputemodule | sort | head -n 1)}"
[[ -n "${LOCAL_BUILD_EXE:-}" && -f "$LOCAL_BUILD_EXE" ]] || die "aperturecomputemodule executable not found under $HOME/DASPi/software/build/aperturecomputemodule"
LOCAL_DIST_DIR="${LOCAL_DIST_DIR:-$HOME/DASPi/software/dist/aperturecomputemodule}"

REMOTE_BASE_DIR="${REMOTE_BASE_DIR:-$HOME/dist/aperturecomputemodule}"
REMOTE_EXE_DIR="$REMOTE_BASE_DIR"

[[ -f "$LOCAL_BUILD_EXE" ]] || die "aperturecomputemodule executable not found: $LOCAL_BUILD_EXE"

log "Staging aperturecomputemodule into $LOCAL_DIST_DIR"
run_cmd mkdir -p "$LOCAL_DIST_DIR"
run_cmd cp "$LOCAL_BUILD_EXE" "$LOCAL_DIST_DIR/aperturecomputemodule"
run_cmd chmod +x "$LOCAL_DIST_DIR/aperturecomputemodule"

for HOST in "${HOSTS[@]}"; do
    REMOTE="${USER}@${HOST}"

    log "Connecting to $REMOTE, client IP $CLIENT, port $PORT"

    remove_remote_dir "$REMOTE" "$REMOTE_BASE_DIR"
    ensure_remote_dir "$REMOTE" "$REMOTE_BASE_DIR"

    copy_file "$LOCAL_DIST_DIR/aperturecomputemodule" "$REMOTE" "$REMOTE_EXE_DIR/aperturecomputemodule"
    remote_run "$REMOTE" "chmod +x '$REMOTE_EXE_DIR/aperturecomputemodule'"

    stop_tmux_session "$REMOTE" "aperturecomputemodule"

    REMOTE_RUN_CMD="ulimit -s unlimited && cd '$REMOTE_EXE_DIR' && exec ./aperturecomputemodule $VERBOSE --clientIp=$CLIENT --port=$PORT > aperturecomputemodule.log 2>&1"

    start_tmux_session "$REMOTE" "aperturecomputemodule" "$REMOTE_RUN_CMD"

    CLIENT="$(increment_subnet "$CLIENT")"
done
