#!/usr/bin/env bash
set -euo pipefail

# ============================
# Default configuration
# ============================
REMOTE_USER="${REMOTE_USER:-pmcgrath}"
REMOTE_HOST="${REMOTE_HOST:-computemodule000}"
REMOTE_PATH="${REMOTE_PATH:-~/dist/computemodule}"
PATTERN="${PATTERN:-output*.bayer}"
LOCAL_DIR="${LOCAL_DIR:-$HOME/bayer_dumps}"

# ============================
# Usage
# ============================
usage() {
    echo "Usage: $0 [-u user] [-h host] [-r remote_path] [-p pattern] [-l local_dir]"
    echo
    echo "Defaults:"
    echo "  user        = $REMOTE_USER"
    echo "  host        = $REMOTE_HOST"
    echo "  remote_path = $REMOTE_PATH"
    echo "  pattern     = $PATTERN"
    echo "  local_dir   = $LOCAL_DIR"
    exit 1
}

# ============================
# Parse args
# ============================
while getopts "u:h:r:p:l:" opt; do
    case "$opt" in
        u) REMOTE_USER="$OPTARG" ;;
        h) REMOTE_HOST="$OPTARG" ;;
        r) REMOTE_PATH="$OPTARG" ;;
        p) PATTERN="$OPTARG" ;;
        l) LOCAL_DIR="$OPTARG" ;;
        *) usage ;;
    esac
done

# ============================
# Run
# ============================
echo "[INFO] Fetching files..."
echo "  From: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${PATTERN}"
echo "  To  : ${LOCAL_DIR}"

mkdir -p "${LOCAL_DIR}"

rsync -avz --progress \
    "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${PATTERN}" \
    "${LOCAL_DIR}/"

echo "[INFO] Done."
