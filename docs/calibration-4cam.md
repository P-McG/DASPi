# DASPi: collect ChArUco calibration for all four cameras

## What this implements

All four Raspberry Pi 5 `ApertureComputeModule` programs start at the same time for each **stationary** board placement. Cameras which cannot see the ChArUco board are **skipped, not treated as failures**. Existing node binaries write native-pixel ChArUco observations to `/tmp/daspi-charuco-observations-moduleN.csv`, and this coordinator copies acceptable files to the laptop.

This is a **one-session, position-and-capture** workflow, not a continuous-running detector between placements. It does **not require recompiling** the Raspberry Pi programs. The script starts/stops the four capture processes automatically for each board pose.

Physical board: **15×15 squares, 30 mm squares, 23 mm markers, DICT_4X4_250**. Sensor image: **1456×1088**. Edit both Pi-side detector and solver if these actual dimensions change.

## 1. Install scripts on the laptop

Copy the three scripts into `~/DASPi/software/scripts/`:

- `calibrate-all-cameras.sh`
- `select-charuco-samples.py`
- `solve-all-camera-intrinsics.sh`

Then:

```bash
chmod +x ~/DASPi/software/scripts/calibrate-all-cameras.sh \
  ~/DASPi/software/scripts/select-charuco-samples.py \
  ~/DASPi/software/scripts/solve-all-camera-intrinsics.sh
```

The script uses the CURRENT mapping:

| Logical camera | Raspberry Pi IP | SSH username | UDP port |
|---|---|---|---|
| 0 | 10.0.2.4 | daspi | 5000 |
| 1 | 10.0.2.5 | daspi | 5000 |
| 2 | 10.0.2.6 | daspi | 5000 |
| 3 | 10.0.2.7 | daspi | 5000 |

**Warning:** The old `seam_image_collection_calibration.sh` uses a swapped 0/1 mapping, separate ports, and requires BOTH cameras to produce data for every pose. Do not use it for this session.

Before proceeding, confirm the current mapping in your launcher (module index = final IP octet − 4), and verify passwordless `ssh daspi@10.0.2.4` etc., and passwordless `sudo systemctl` on each Pi.

## 2. Collect all four cameras concurrently

```bash
SESSION=calib4_001 SECONDS_PER_POSE=12 MIN_CORNERS_PER_FRAME=20 \
  ~/DASPi/software/scripts/calibrate-all-cameras.sh
```

For each board placement: move the board, hold it still, press **Enter**. The laptop starts all four nodes in parallel, waits up to 12 seconds, then accepts the data from whichever cameras saw enough corners. Invisible cameras are **normal** and don't force retries. Type `q` at the prompt to finish. The script restores regular `aperturecomputemodule` services that were running at the beginning.

Repeat using the same `SESSION=calib4_001` later to **add more poses** without overwriting successful prior poses. Give each real-world stationary board placement its own pose ID. Aim for at least 15–25 distinct board locations/orientations for EACH camera, including board corners near the sensor image edges and varied board tilt. Avoid repeated views of exactly the same placement.

Output is in:

```text
~/DASPi/software/config/charuco-captures/calib4_001/
```

The script prints a *shared-pose matrix*, showing how many stationary board placements were simultaneously observed by each camera pair. To estimate inter-camera alignment, the set of shared-pose pairs must form a connected graph across camera 0, 1, 2 and 3. The board need NEVER be visible to all four cameras in one placement.

## 3. Build and solve intrinsics for all four modules

Build the laptop solver, if necessary:

```bash
meson compile -C ~/DASPi/software/build/computemodule-native
```

Review the CSV summary at the end of capture. The current C++ solver demands >=10 usable independent views per camera; more diverse views are preferable. Then:

```bash
~/DASPi/software/scripts/solve-all-camera-intrinsics.sh calib4_001
```

Generates *candidate* files:

```text
.../calib4_001/camera-intrinsics-module0.yml
.../calib4_001/camera-intrinsics-module1.yml
.../calib4_001/camera-intrinsics-module2.yml
.../calib4_001/camera-intrinsics-module3.yml
.../calib4_001/charuco-pose-report.csv
.../calib4_001/relative-camera-pose-report-0-1-only.csv
.../calib4_001/solve.log
```

**Review each camera's calibration RMS and camera matrix** before deployment. The existing pinhole + distortion calibration (`cv::calibrateCamera`) may struggle at the edges of a very wide/fisheye lens. Diverse views and held-out reprojection tests are important; compare error across the image, not just the aggregate RMS.

## 4. Relative mounting rotations (extrinsics) — not solved for all four yet

The existing `daspi-calibrate-charuco.cpp` already solves *intrinsics by module ID*. However, `WriteRelativePoseReport`, `ComputeAverageObservedRelativeTransform` and `WriteCalibrationCorrectionFile` presently use ONLY modules 0 and 1. `--nominalRelativeRvecDeg` also represents only a single camera pair.

Therefore this setup **does not** produce a valid four-camera `camera-calibration.txt` yet. Do **not** use `--writeCalibration` to replace your current file. The next source-code change is an N-camera relative-pose graph / bundle-adjustment solver, with nominal directions obtained from the DASPi rig geometry. Its inputs will be the same collected CSVs: no need to redo a successful capture if enough shared board poses were taken.

To determine extrinsics without every camera seeing the board at once, gather stationary shared-pose observations for a connected set of camera pairs (e.g. 0–1, 1–2, 2–3, but the actual viable pairs depend on the rig). Each pair should ideally have several geometrically varied shared placements. If a pair cannot observe the board simultaneously, use a different connecting pair, a smaller board, a fixed reference, or an accurately surveyed fixture pose. Merely showing a moved board to each camera at different times does not determine their relative orientations.

## 5. Deploy only after review

This is *not* performed automatically. Back up each deployed YAML (if present), then copy its **matching module** intrinsics file into `/opt/daspi/config/` on that node. Ensure `--cameraIntrinsicsPrefix=/opt/daspi/config/camera-intrinsics` is actually appended to `ARGS` by the systemd launcher; setting a variable without adding the CLI option will not load it. Restart the capture services and inspect the runtime logs for confirmation.

Keep the existing `camera-calibration.txt` (yaw/pitch/roll corrections) unchanged until the multi-camera extrinsic solver is implemented, validated, and reviewed.

## Troubleshooting: old CAM3 default/0 recordings (fix added Oct 23, 2026)

Earlier script releases launched `rm -f` for each remote CSV in the background and
did not check whether the deletion actually succeeded. A Pi capture service can
leave a root-owned file in `/tmp`, which the unprivileged SSH user may not be able
to remove. As a result, an old file with `session_id=default,pose_id=0` can be copied
and rejected as if the new camera had poor ChArUco detection.

The updated collector now uses `sudo rm -f` separately for each module, confirms
that the old file no longer exists, and aborts on any session/pose/module mismatch.
It never silently accepts an old CSV. It also resumes with the next UNUSED pose
number even if the previous attempt was rejected.

Before resuming, inspect the camera node's CSV ownership:

```bash
ssh daspi@10.0.2.7 'stat -c "%U:%G %a %y %n" /tmp/daspi-charuco-observations-module3.csv 2>/dev/null || echo "no CSV present"'
```

To install this corrected version, replace ONLY the collector script on the
laptop, keep the preexisting session folder, and run with
`SESSION=calib4_001`. Do not rename stale observations to a new pose or inject
old `default,0` files into the combined calibration data: the physical board
position is not verified.

If the revised script reports that CAM3 has **no detections** after a verified
clean restart, then image visibility/focus/exposure still needs investigation.
The historical 86-corner detection is evidence only that CAM3 previously saw
the board successfully, not proof that it was visible in the new captures.
