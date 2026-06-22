#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-calib_001}"
POSES="${POSES:-12}"
SECONDS_PER_POSE="${SECONDS_PER_POSE:-35}"
MIN_ROWS_PER_POSE="${MIN_ROWS_PER_POSE:-40}"
MIN_CORNERS_PER_FRAME="${MIN_CORNERS_PER_FRAME:-20}"
MAX_RETRIES_PER_POSE="${MAX_RETRIES_PER_POSE:-5}"

# Current swapped logical mapping:
# logical module 0 -> 10.0.2.5
# logical module 1 -> 10.0.2.4
AP0="${AP0:-10.0.2.5}"
AP1="${AP1:-10.0.2.4}"

PORT0="${PORT0:-5010}"
PORT1="${PORT1:-5000}"

CLIENT_IP="${CLIENT_IP:-10.0.2.1}"
CAL="${CAL:-/opt/daspi/config/camera-calibration.txt}"

OUTDIR="$HOME/DASPi/software/config/charuco-captures"
COMBINED0="$HOME/DASPi/software/config/daspi-charuco-observations-module0.csv"
COMBINED1="$HOME/DASPi/software/config/daspi-charuco-observations-module1.csv"

mkdir -p "$OUTDIR"

echo "Session: $SESSION"
echo "Poses: $POSES"
echo "Seconds per pose: $SECONDS_PER_POSE"
echo "Minimum rows per pose: $MIN_ROWS_PER_POSE"
echo "Minimum corners per frame: $MIN_CORNERS_PER_FRAME"
echo "Max retries per pose: $MAX_RETRIES_PER_POSE"
echo "AP0: $AP0 port $PORT0"
echo "AP1: $AP1 port $PORT1"
echo

cleanup_remote_node() {
    local label="$1"
    local host="$2"

    ssh "daspi@$host" "LABEL='$label' bash -s" <<'REMOTE'
set -euo pipefail

sudo systemctl stop aperturecomputemodule 2>/dev/null || true
sudo pkill -x aperturecomputemodule 2>/dev/null || true
sudo pkill -f '/opt/daspi/aperturecomputemodule' 2>/dev/null || true

sleep 1

sudo rm -f /tmp/daspi-charuco-observations-module*.csv

if ls /tmp/daspi-charuco-observations-module*.csv >/dev/null 2>&1; then
    echo "ERROR: ${LABEL} CSV cleanup failed"
    ls -lh /tmp/daspi-charuco-observations-module*.csv
    exit 1
fi

exit 0
REMOTE
}

wait_for_fresh_csv() {
    local label="$1"
    local host="$2"
    local module="$3"
    local expected="$4"

    ssh "daspi@$host" "LABEL='$label' MODULE='$module' EXPECTED='$expected' bash -s" <<'REMOTE'
set -euo pipefail

file="/tmp/daspi-charuco-observations-module${MODULE}.csv"

for i in $(seq 1 20); do
    if [ -s "$file" ]; then
        id=$(awk -F, 'NF==9 && $1!="session_id" && $1 !~ /^#/ {print $1 "," $2; exit}' "$file")
        if [ "$id" = "$EXPECTED" ]; then
            echo "$LABEL fresh CSV OK: $id"
            exit 0
        fi
    fi
    sleep 1
done

echo "ERROR: $LABEL did not produce fresh CSV for expected pose $EXPECTED"
echo "Current $LABEL CSV ID:"
awk -F, 'NF==9 && $1!="session_id" && $1 !~ /^#/ {print $1 "," $2; exit}' "$file" 2>/dev/null || true
exit 1
REMOTE
}

check_remote_csv_quality() {
    local label="$1"
    local host="$2"
    local module="$3"
    local expected="$4"

    ssh "daspi@$host" \
        "LABEL='$label' MODULE='$module' EXPECTED='$expected' MIN_ROWS='$MIN_ROWS_PER_POSE' MIN_CORNERS='$MIN_CORNERS_PER_FRAME' bash -s" <<'REMOTE'
set -euo pipefail

file="/tmp/daspi-charuco-observations-module${MODULE}.csv"

rows=$(awk -F, '
NF==9 && $1!="session_id" && $1 !~ /^#/ {
    n++
}
END {
    print n+0
}' "$file")

max_corners=$(awk -F, '
NF==9 && $1!="session_id" && $1 !~ /^#/ {
    frame=$4
    corner=$7
    seen[frame "," corner]=1
}
END {
    for (k in seen) {
        split(k, a, ",")
        frame=a[1]
        count[frame]++
    }

    max=0
    for (f in count) {
        if (count[f] > max) {
            max=count[f]
        }
    }

    print max+0
}' "$file")

id=$(awk -F, 'NF==9 && $1!="session_id" && $1 !~ /^#/ {print $1 "," $2; exit}' "$file")

echo "$LABEL id=$id rows=$rows max_unique_corners_in_frame=$max_corners"

if [ "$id" != "$EXPECTED" ]; then
    echo "ERROR: $LABEL wrong pose ID: $id expected $EXPECTED"
    exit 1
fi

if [ "$rows" -lt "$MIN_ROWS" ]; then
    echo "ERROR: $LABEL too few ChArUco rows: $rows < $MIN_ROWS"
    exit 1
fi

if [ "$max_corners" -lt "$MIN_CORNERS" ]; then
    echo "ERROR: $LABEL no frame has enough ChArUco corners: max=$max_corners < $MIN_CORNERS"
    exit 1
fi

exit 0
REMOTE
}

print_remote_header() {
    local label="$1"
    local host="$2"
    local module="$3"

    ssh "daspi@$host" "LABEL='$label' MODULE='$module' bash -s" <<'REMOTE'
set -euo pipefail

file="/tmp/daspi-charuco-observations-module${MODULE}.csv"

echo "$LABEL"
head -12 "$file" 2>/dev/null || true
REMOTE
}

copy_remote_csv() {
    local label="$1"
    local host="$2"
    local module="$3"
    local dst="$4"

    local remote="/tmp/daspi-charuco-observations-module${module}.csv"

    if ssh "daspi@$host" "test -s '$remote'"; then
        scp "daspi@$host:$remote" "$dst"
        echo "$label copied -> $dst"
    else
        echo "ERROR: no $label CSV at $remote"
        exit 1
    fi
}

POSE=0

while [ "$POSE" -lt "$POSES" ]; do
    ATTEMPT=1
    POSE_OK=0

    while [ "$ATTEMPT" -le "$MAX_RETRIES_PER_POSE" ]; do
        echo
        echo "============================================================"
        echo "Pose $POSE / $((POSES - 1)) attempt $ATTEMPT / $MAX_RETRIES_PER_POSE"
        echo "Place and hold the ChArUco board where BOTH cameras see it."
        read -r -p "Press Enter to capture pose $POSE..."

        LOG0="$OUTDIR/${SESSION}-pose${POSE}-attempt${ATTEMPT}-module0.log"
        LOG1="$OUTDIR/${SESSION}-pose${POSE}-attempt${ATTEMPT}-module1.log"

        CSV0_TMP="$OUTDIR/${SESSION}-pose${POSE}-attempt${ATTEMPT}-module0.csv"
        CSV1_TMP="$OUTDIR/${SESSION}-pose${POSE}-attempt${ATTEMPT}-module1.csv"

        CSV0_FINAL="$OUTDIR/${SESSION}-pose${POSE}-module0.csv"
        CSV1_FINAL="$OUTDIR/${SESSION}-pose${POSE}-module1.csv"

        echo "[pose $POSE attempt $ATTEMPT] stopping old aperture processes and clearing remote CSVs"
        cleanup_remote_node "AP0" "$AP0"
        cleanup_remote_node "AP1" "$AP1"

        echo "[pose $POSE attempt $ATTEMPT] starting both modules"

        ssh "daspi@$AP0" "
          DASPI_CHARUCO_SESSION_ID='$SESSION' \
          DASPI_CHARUCO_POSE_ID='$POSE' \
          timeout '${SECONDS_PER_POSE}s' \
          /opt/daspi/aperturecomputemodule \
            --clientIp='$CLIENT_IP' \
            --port='$PORT0' \
            --moduleIndex=0 \
            --cameraCalibration='$CAL'
        " > "$LOG0" 2>&1 &

        PID0=$!

        ssh "daspi@$AP1" "
          DASPI_CHARUCO_SESSION_ID='$SESSION' \
          DASPI_CHARUCO_POSE_ID='$POSE' \
          timeout '${SECONDS_PER_POSE}s' \
          /opt/daspi/aperturecomputemodule \
            --clientIp='$CLIENT_IP' \
            --port='$PORT1' \
            --moduleIndex=1 \
            --cameraCalibration='$CAL'
        " > "$LOG1" 2>&1 &

        PID1=$!

        wait "$PID0" || true
        wait "$PID1" || true

        echo "[pose $POSE attempt $ATTEMPT] waiting for fresh remote CSVs"

        if ! wait_for_fresh_csv "AP0" "$AP0" "0" "$SESSION,$POSE"; then
            echo "WARNING: AP0 did not produce a fresh CSV for pose $POSE attempt $ATTEMPT"
            echo "AP0 log tail:"
            tail -60 "$LOG0" || true
            ATTEMPT=$((ATTEMPT + 1))
            continue
        fi

        if ! wait_for_fresh_csv "AP1" "$AP1" "1" "$SESSION,$POSE"; then
            echo "WARNING: AP1 did not produce a fresh CSV for pose $POSE attempt $ATTEMPT"
            echo "AP1 log tail:"
            tail -60 "$LOG1" || true
            ATTEMPT=$((ATTEMPT + 1))
            continue
        fi

        echo "[pose $POSE attempt $ATTEMPT] checking CSV quality"

        if ! check_remote_csv_quality "AP0" "$AP0" "0" "$SESSION,$POSE"; then
            echo "WARNING: AP0 CSV quality failed for pose $POSE attempt $ATTEMPT"
            echo "Move the board more into AP0's view, then retry this same pose."
            echo "AP0 log tail:"
            tail -60 "$LOG0" || true
            ATTEMPT=$((ATTEMPT + 1))
            continue
        fi

        if ! check_remote_csv_quality "AP1" "$AP1" "1" "$SESSION,$POSE"; then
            echo "WARNING: AP1 CSV quality failed for pose $POSE attempt $ATTEMPT"
            echo "Move the board more into AP1's view, then retry this same pose."
            echo "AP1 log tail:"
            tail -60 "$LOG1" || true
            ATTEMPT=$((ATTEMPT + 1))
            continue
        fi

        echo "[pose $POSE attempt $ATTEMPT] remote CSV headers"
        print_remote_header "AP0=$AP0" "$AP0" "0"
        print_remote_header "AP1=$AP1" "$AP1" "1"

        echo "[pose $POSE attempt $ATTEMPT] copying CSVs"

        copy_remote_csv "[pose $POSE attempt $ATTEMPT] module0" "$AP0" "0" "$CSV0_TMP"
        copy_remote_csv "[pose $POSE attempt $ATTEMPT] module1" "$AP1" "1" "$CSV1_TMP"

        cp "$CSV0_TMP" "$CSV0_FINAL"
        cp "$CSV1_TMP" "$CSV1_FINAL"

        echo "[pose $POSE] accepted attempt $ATTEMPT"
        echo "[pose $POSE] final module0 CSV: $CSV0_FINAL"
        echo "[pose $POSE] final module1 CSV: $CSV1_FINAL"

        POSE_OK=1
        break
    done

    if [ "$POSE_OK" -ne 1 ]; then
        echo
        echo "ERROR: pose $POSE failed after $MAX_RETRIES_PER_POSE attempts."
        echo "Stopping so bad data is not included."
        exit 1
    fi

    POSE=$((POSE + 1))
done

echo
echo "Recombining clean CSVs..."

{
    echo "session_id,pose_id,timestamp_ns,frame,module,facet,corner_id,x,y"
    awk -F, 'NF==9 && $1!="session_id" && $1 !~ /^#/ && $2!=""' \
        "$OUTDIR/${SESSION}"-pose*-module0.csv 2>/dev/null || true
} > "$COMBINED0"

{
    echo "session_id,pose_id,timestamp_ns,frame,module,facet,corner_id,x,y"
    awk -F, 'NF==9 && $1!="session_id" && $1 !~ /^#/ && $2!=""' \
        "$OUTDIR/${SESSION}"-pose*-module1.csv 2>/dev/null || true
} > "$COMBINED1"

echo
echo "Combined files:"
wc -l "$COMBINED0" "$COMBINED1"

echo
echo "Pose row counts, module 0:"
awk -F, 'NR>1 {print $2}' "$COMBINED0" | sort -n | uniq -c

echo
echo "Pose row counts, module 1:"
awk -F, 'NR>1 {print $2}' "$COMBINED1" | sort -n | uniq -c

echo
echo "Shared pose IDs:"
comm -12 \
  <(awk -F, 'NR>1 {print $1","$2}' "$COMBINED0" | sort -u) \
  <(awk -F, 'NR>1 {print $1","$2}' "$COMBINED1" | sort -u)

echo
echo "Done."
