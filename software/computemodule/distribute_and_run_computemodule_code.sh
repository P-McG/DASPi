#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./distribute_and_run_computemodule_code.sh
#   ./distribute_and_run_computemodule_code.sh host1 host2
#
# Optional environment overrides:
#   REMOTE_BASE_DIR
#   LOCAL_BUILD_DIR
#   REMOTE_RUN_CMD

HOSTS=("$@")
if [ "${#HOSTS[@]}" -eq 0 ]; then
    HOSTS=("computemodule000")
fi

REMOTE_BASE_DIR="${REMOTE_BASE_DIR:-$HOME/DASPi/computemodule}"
LOCAL_BUILD_DIR="${LOCAL_BUILD_DIR:-$HOME/DASPi/src/computemodule/trial_1f}"
REMOTE_RUN_CMD="${REMOTE_RUN_CMD:-./trial --verbose --nApertureComputeModules=2 --usbSubnets='10.0.2.0' --port=5000}"

for HOST in "${HOSTS[@]}"; do
    echo "Connecting to ${USER}@${HOST}..."

    ssh "${USER}@${HOST}" "echo 'Connected to ${HOST}'"

    ssh "${USER}@${HOST}" "rm -rf '${REMOTE_BASE_DIR}'"
    scp -r "${LOCAL_BUILD_DIR}" "${USER}@${HOST}:${REMOTE_BASE_DIR}"

    ssh -t "${USER}@${HOST}" "cd '${REMOTE_BASE_DIR}/bin' && ${REMOTE_RUN_CMD}"

    echo "Finished with ${HOST}"
    echo "-----------------------"
done
