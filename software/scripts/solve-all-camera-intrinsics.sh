#!/usr/bin/env bash
# Solve intrinsic parameters for all four cameras, using collected ChArUco data.
# Does NOT write or deploy camera-calibration.txt (multi-camera extrinsics need more code).
set -euo pipefail

SESSION="${1:?Usage: $0 SESSION_NAME}"
MIN_VIEWS="${MIN_VIEWS:-10}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/DASPi/software/config/charuco-captures}"
OUTDIR="$OUTPUT_ROOT/$SESSION"
SOLVER="${SOLVER:-$HOME/DASPi/software/build/computemodule-native/computemodule/daspi-calibrate-charuco}"

[[ -x "$SOLVER" ]] || { echo "ERROR: solver not executable: $SOLVER" >&2; exit 1; }
files=()
for m in 0 1 2 3; do
  f="$OUTDIR/daspi-charuco-observations-module${m}.csv"
  [[ -s "$f" ]] || { echo "ERROR: no observations CSV for camera $m: $f" >&2; exit 1; }
  count=$(awk -F, 'NR>1 && NF==9 {seen[$1 SUBSEP $2]=1} END {for (key in seen) ++n; print n+0}' "$f")
  echo "Camera $m: $count distinct board poses"
  if ((count < MIN_VIEWS)); then
    echo "ERROR: camera $m has fewer than $MIN_VIEWS independent views; do not fit yet." >&2
    exit 1
  fi
  files+=("$f")
done

observations=$(IFS=,; echo "${files[*]}")
"$SOLVER" \
  --observations="$observations" \
  --poseCsv="$OUTDIR/charuco-pose-report.csv" \
  --relativePoseCsv="$OUTDIR/relative-camera-pose-report-0-1-only.csv" \
  --squaresX=15 \
  --squaresY=15 \
  --squareLength=0.030 \
  --markerLength=0.023 \
  --dictionary=DICT_4X4_250 \
  --fx=600 --fy=600 --cx=728 --cy=544 \
  --minCorners=20 \
  --calibrateIntrinsics \
  --imageWidth=1456 --imageHeight=1088 \
  --intrinsicsPrefix="$OUTDIR/camera-intrinsics" \
  2>&1 | tee "$OUTDIR/solve.log"

echo
for m in 0 1 2 3; do
  test -s "$OUTDIR/camera-intrinsics-module${m}.yml" || {
    echo "ERROR: solver did not create module$m.yml" >&2
    exit 1
  }
done
echo "Four candidate intrinsics YAML files are in $OUTDIR"
echo "Review the RMS/focal lengths; do not deploy until checked."
echo "Note: existing extrinsic-calibration writer only handles modules 0 and 1."
