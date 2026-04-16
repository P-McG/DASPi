#!/usr/bin/env bash
set -Eeuo pipefail

HOSTNAME_NOW="$(hostname)"

CLIENT_IP="${CLIENT_IP:-10.0.2.2}"
BASE_PORT="${BASE_PORT:-5000}"
VERBOSE="${VERBOSE:---verbose}"
BINARY="${BINARY:-/opt/daspi/aperturecomputemodule}"

increment_by_host_index() {
    local index="$1"
    echo $((BASE_PORT + index * 2))
}

case "$HOSTNAME_NOW" in
    aperturecomputemodule000)
        PORT="$(increment_by_host_index 0)"
        ;;
    aperturecomputemodule001)
        PORT="$(increment_by_host_index 1)"
        ;;
    *)
        echo "Unknown hostname '$HOSTNAME_NOW'; defaulting PORT=$BASE_PORT" >&2
        PORT="$BASE_PORT"
        ;;
esac

mkdir -p /var/log/aperturecomputemodule

ulimit -s unlimited
exec "$BINARY" $VERBOSE --clientIp="$CLIENT_IP" --port="$PORT"
