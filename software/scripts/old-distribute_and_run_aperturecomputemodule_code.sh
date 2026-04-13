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

BASE_PORT="${PORT:-5000}"
CLIENT_IP="${CLIENT_IP:-10.0.2.2}"
BASE_SERVER_IP="${BASE_SERVER_IP:-10.0.2.3}"
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

increment_host() {
    local ip="$1"
    local delta="${2:-1}"
    local o1 o2 o3 o4

    IFS='.' read -r o1 o2 o3 o4 <<< "$ip"

    [[ -n "${o1:-}" && -n "${o2:-}" && -n "${o3:-}" && -n "${o4:-}" ]] \
        || die "Invalid IPv4 address: $ip"

    o4=$((o4 + delta))
    (( o4 >= 0 && o4 <= 254 )) || die "IPv4 host overflow for $ip + $delta"

    echo "$o1.$o2.$o3.$o4"
}

frame_port_for_index() {
    local index="$1"
    echo $((BASE_PORT + index * 2))
}

control_port_for_frame() {
    local frame_port="$1"
    echo $((frame_port + 1))
}

server_ip_for_index() {
    local index="$1"
    increment_host "$BASE_SERVER_IP" "$index"
}

stage_local_binary() {
    log "Staging aperturecomputemodule into $LOCAL_DIST_DIR"
    run_cmd mkdir -p "$LOCAL_DIST_DIR"
    run_cmd cp "$LOCAL_BUILD_EXE" "$LOCAL_DIST_DIR/aperturecomputemodule"
    run_cmd chmod +x "$LOCAL_DIST_DIR/aperturecomputemodule"
}

deploy_binary_to_remote() {
    local remote="$1"

    remove_remote_dir "$remote" "$REMOTE_BASE_DIR"
    ensure_remote_dir "$remote" "$REMOTE_BASE_DIR"

    copy_file "$LOCAL_DIST_DIR/aperturecomputemodule" "$remote" "$REMOTE_EXE_DIR/aperturecomputemodule"
    remote_run "$remote" "chmod +x '$REMOTE_EXE_DIR/aperturecomputemodule'"
}

start_remote_process() {
    local remote="$1"
    local client_ip="$2"
    local frame_port="$3"

    local run_cmd_str
    run_cmd_str="ulimit -s unlimited && cd '$REMOTE_EXE_DIR' && exec ./aperturecomputemodule $VERBOSE --clientIp=$client_ip --port=$frame_port > aperturecomputemodule.log 2>&1"

    stop_tmux_session "$remote" "aperturecomputemodule"
    start_tmux_session "$remote" "aperturecomputemodule" "$run_cmd_str"
}

print_plan() {
    log "Deployment plan:"
    log "  client IP      : $CLIENT_IP"
    log "  base server IP : $BASE_SERVER_IP"
    log "  base frame port: $BASE_PORT"

    local i
    for i in "${!HOSTS[@]}"; do
        local host="${HOSTS[$i]}"
        local server_ip
        local frame_port
        local control_port

        server_ip="$(server_ip_for_index "$i")"
        frame_port="$(frame_port_for_index "$i")"
        control_port="$(control_port_for_frame "$frame_port")"

        log "  [$i] host=$host serverIp=$server_ip framePort=$frame_port controlPort=$control_port"
    done
}

stage_local_binary
print_plan

for i in "${!HOSTS[@]}"; do
    HOST="${HOSTS[$i]}"
    REMOTE="${USER}@${HOST}"

    SERVER_IP="$(server_ip_for_index "$i")"
    FRAME_PORT="$(frame_port_for_index "$i")"
    CONTROL_PORT="$(control_port_for_frame "$FRAME_PORT")"

    log "Connecting to $REMOTE"
    log "  client IP   : $CLIENT_IP"
    log "  server IP   : $SERVER_IP"
    log "  frame port  : $FRAME_PORT"
    log "  control port: $CONTROL_PORT"

    deploy_binary_to_remote "$REMOTE"
    start_remote_process "$REMOTE" "$CLIENT_IP" "$FRAME_PORT"
done
