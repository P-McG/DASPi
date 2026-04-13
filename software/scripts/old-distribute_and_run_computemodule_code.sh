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

if [[ "${#HOSTS[@]}" -eq 0 ]]; then
    HOSTS=("computemodule000")
fi

LOCAL_BUILD_EXE="${LOCAL_BUILD_EXE:-$(find "$HOME/DASPi/software/build/computemodule" -type f -executable -name computemodule | sort | head -n 1)}"
[[ -n "${LOCAL_BUILD_EXE:-}" && -f "$LOCAL_BUILD_EXE" ]] || die "computemodule executable not found under $HOME/DASPi/software/build/computemodule"
LOCAL_DIST_DIR="${LOCAL_DIST_DIR:-$HOME/DASPi/software/dist/computemodule}"

REMOTE_BASE_DIR="${REMOTE_BASE_DIR:-$HOME/dist/computemodule}"
REMOTE_EXE_DIR="$REMOTE_BASE_DIR"

N_APERTURE_MODULES="${N_APERTURE_MODULES:-2}"
USB_SUBNETS="${USB_SUBNETS:-10.0.2.0}"
PORT="${PORT:-5000}"
VERBOSE="${VERBOSE:---verbose}"

REMOTE_RUN_CMD="${REMOTE_RUN_CMD:-./computemodule $VERBOSE --nApertureComputeModules=$N_APERTURE_MODULES --usbSubnets=$USB_SUBNETS --port=$PORT}"

[[ -f "$LOCAL_BUILD_EXE" ]] || die "computemodule executable not found: $LOCAL_BUILD_EXE"

log "Staging computemodule into $LOCAL_DIST_DIR"
run_cmd mkdir -p "$LOCAL_DIST_DIR"
run_cmd cp "$LOCAL_BUILD_EXE" "$LOCAL_DIST_DIR/computemodule"
run_cmd chmod +x "$LOCAL_DIST_DIR/computemodule"

for HOST in "${HOSTS[@]}"; do
    REMOTE="${USER}@${HOST}"

    log "Connecting to $REMOTE"
    remote_run "$REMOTE" "echo 'Connected to $HOST'"

    log "Replacing remote directory $REMOTE_BASE_DIR"
    remove_remote_dir "$REMOTE" "$REMOTE_BASE_DIR"
    ensure_remote_dir "$REMOTE" "$REMOTE_BASE_DIR"

    log "Copying $LOCAL_DIST_DIR to $REMOTE:$REMOTE_BASE_DIR"
    copy_file "$LOCAL_DIST_DIR/computemodule" "$REMOTE" "$REMOTE_EXE_DIR/computemodule"

    log "Stopping old computemodule tmux session"
    stop_tmux_session "$REMOTE" "computemodule"

    log "Starting computemodule"
    start_tmux_session "$REMOTE" "computemodule" \
        "cd '$REMOTE_EXE_DIR' && exec $REMOTE_RUN_CMD > computemodule.log 2>&1"

    log "Finished with $HOST"
    echo "-----------------------"
done
