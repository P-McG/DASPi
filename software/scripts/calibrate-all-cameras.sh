#!/usr/bin/env bash
# Collect ChArUco observations from all four DASPi cameras concurrently.
# Cameras need NOT all see the board in the same pose.
# Run from the laptop; no Pi binary changes required.
set -euo pipefail

SESSION="${SESSION:-calib4_$(date +%Y%m%d_%H%M%S)}"
SECONDS_PER_POSE="${SECONDS_PER_POSE:-12}"
MIN_CORNERS_PER_FRAME="${MIN_CORNERS_PER_FRAME:-20}"
MIN_GOOD_FRAMES="${MIN_GOOD_FRAMES:-2}"
CLIENT_IP="${CLIENT_IP:-10.0.2.1}"
PORT="${PORT:-5000}"
CAL="${CAL:-/opt/daspi/config/camera-calibration.txt}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/DASPi/software/config/charuco-captures}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTDIR="$OUTPUT_ROOT/$SESSION"
HOSTS=(10.0.2.4 10.0.2.5 10.0.2.6 10.0.2.7)
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)
SCP=(scp -q -o BatchMode=yes -o ConnectTimeout=8)
mkdir -p "$OUTDIR"

if [[ ! "$SESSION" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo "ERROR: SESSION must contain only letters, numbers, underscore and hyphen." >&2
  exit 1
fi
for v in SECONDS_PER_POSE MIN_CORNERS_PER_FRAME MIN_GOOD_FRAMES PORT; do
  [[ "${!v}" =~ ^[0-9]+$ ]] || { echo "ERROR: $v must be an integer" >&2; exit 1; }
done

restore_services() {
  # Restart ONLY services that were active before calibration.
  trap - EXIT
  for m in 0 1 2 3; do
    if [[ -f "$OUTDIR/.service-was-active-$m" ]]; then
      "${SSH[@]}" "daspi@${HOSTS[$m]}" 'sudo systemctl start aperturecomputemodule' || true
    fi
  done
}
trap restore_services EXIT

printf 'Session: %s\nOutput: %s\n' "$SESSION" "$OUTDIR"
echo "Camera mappings: .4→0, .5→1, .6→2, .7→3 (port $PORT on all)"
echo 'Stopping regular capture services for calibration...'
# Session resumption must not reuse markers from a previous invocation.
rm -f "$OUTDIR"/.service-was-active-{0,1,2,3}
for m in 0 1 2 3; do
  host="${HOSTS[$m]}"
  if "${SSH[@]}" "daspi@$host" 'systemctl is-active --quiet aperturecomputemodule'; then
    touch "$OUTDIR/.service-was-active-$m"
  fi
  "${SSH[@]}" "daspi@$host" 'sudo systemctl stop aperturecomputemodule'
done

# Find the first unused pose number, including any previously REJECTED captures.
# This avoids overwriting old logs or rejected files when resuming a session.
pose=0
for file in "$OUTDIR"/"$SESSION"-pose*-module*; do
  [[ -e "$file" ]] || continue
  base="$(basename "$file")"
  if [[ "$base" =~ ^${SESSION}-pose([0-9]+)-module[0-3] ]]; then
    num=${BASH_REMATCH[1]}
    (( 10#$num >= pose )) && pose=$((10#$num + 1))
  fi
done

echo
echo "For each pose, hold the board STILL and press Enter."
echo "Only cameras seeing the board are collected; type q and Enter to finish."
echo "Aim for 15–25 DISTINCT board positions/tilts per camera and shared poses"
echo "between camera pairs where possible."
while true; do
  echo
  read -r -p "Pose $pose: board positioned? [Enter=capture / q=finish] " reply || break
  [[ "${reply,,}" == 'q' ]] && break

  echo "Preparing all four nodes and verifying old CSVs are gone..."
  for m in 0 1 2 3; do
    host="${HOSTS[$m]}"
    remote="/tmp/daspi-charuco-observations-module${m}.csv"

    # The regular service may have created a root-owned /tmp CSV.
    # Delete with sudo and REQUIRE a successful absence check. Avoid
    # starting a calibration against stale data (the original script's bug).
    if ! "${SSH[@]}" "daspi@$host" \
      "sudo rm -f '$remote' && test ! -e '$remote'"; then
      echo "ERROR: CAM$m ($host) cannot clear $remote" >&2
      echo "Inspect ownership, /tmp permissions, and any still-running camera service." >&2
      exit 1
    fi
  done

  pids=()
  for m in 0 1 2 3; do
    host="${HOSTS[$m]}"
    # Environment values are validated above; all shell arguments are quoted.
    remote="DASPI_CHARUCO_SESSION_ID='$SESSION' DASPI_CHARUCO_POSE_ID='$pose' \
      timeout --signal=TERM --kill-after=3s '${SECONDS_PER_POSE}s' \
      /opt/daspi/aperturecomputemodule \
      --clientIp='$CLIENT_IP' --port='$PORT' --moduleIndex='$m' \
      --cameraCalibration='$CAL'"
    "${SSH[@]}" "daspi@$host" "$remote" \
      >"$OUTDIR/$SESSION-pose${pose}-module$m.log" 2>&1 &
    pids+=("$!")
  done
  # timeout normally returns 124. This is expected; check CSVs instead.
  for pid in "${pids[@]}"; do wait "$pid" || true; done

  accepted=0
  echo "Results for pose $pose:"
  for m in 0 1 2 3; do
    host="${HOSTS[$m]}"
    remote="/tmp/daspi-charuco-observations-module${m}.csv"
    dest="$OUTDIR/$SESSION-pose${pose}-module${m}.csv"
    if ! "${SSH[@]}" "daspi@$host" "test -s '$remote'"; then
      echo "  Camera $m: not visible / no detections (OK)"
      continue
    fi
    "${SCP[@]}" "daspi@$host:$remote" "$dest"

    # A current capture MUST contain the requested session, pose and camera IDs.
    # Never count legacy 'default,0' observations as a new calibration pose.
    # All observation rows must match; mixed/stale data are rejected outright.
    if ! awk -F, -v sid="$SESSION" -v pid="$pose" -v mod="$m" '
        NF == 9 && $1 != "session_id" && $1 !~ /^#/ {
          count++
          if ($1 != sid || $2 != pid || $5 != mod) bad++
        }
        END { exit (count > 0 && bad == 0 ? 0 : 1) }
      ' "$dest"; then
      echo "  Camera $m: WRONG session/pose/module metadata (not live data)." >&2
      echo "  First observations:"
      awk -F, 'NF==9 && $1!="session_id" && $1 !~ /^#/ {print "    " $1 "," $2 ",module=" $5; exit}' "$dest"
      mv "$dest" "${dest%.csv}.rejected.csv"
      echo "  Check CAM$m launch environment and stale /tmp CSV ownership." >&2
      # Stop this run: a recording failure should not be hidden as poor
      # marker visibility, and the operator should not keep moving the board.
      exit 1
    fi

    # Count distinct frames with at least MIN_CORNERS_PER_FRAME distinct corners.
    good=$(awk -F, -v min="$MIN_CORNERS_PER_FRAME" -v sid="$SESSION" -v pid="$pose" -v mod="$m" '
      NF == 9 && $1==sid && $2==pid && $5==mod && $7 ~ /^[0-9]+$/ {
        key=$4 SUBSEP $7; if (!(key in seen)) { seen[key]=1; count[$4]++ }
      }
      END { for (frame in count) if (count[frame] >= min) ++n; print n+0 }
    ' "$dest")
    if ((good >= MIN_GOOD_FRAMES)); then
      echo "  Camera $m: accepted ($good good frames)"
      accepted=$((accepted+1))
    else
      echo "  Camera $m: insufficient corners/frames ($good good frames; ignored)"
      mv "$dest" "${dest%.csv}.rejected.csv"
    fi
  done

  if ((accepted == 0)); then
    echo "No useful observations; reposition the board before the next pose."
  else
    echo "Pose $pose saved for $accepted camera(s)."
  fi
  # Never overwrite prior logs/rejections even when no cameras accepted.
  pose=$((pose+1))
done

echo
echo "Building balanced, one-frame-per-pose observation CSV files..."
python3 "$SCRIPT_DIR/select-charuco-samples.py" \
  --capture-dir "$OUTDIR" --session "$SESSION" --min-corners "$MIN_CORNERS_PER_FRAME"

echo
echo "Completed capture. CSVs and diagnostic reports are in: $OUTDIR"
echo "Original capture services will be restored if they were active."
