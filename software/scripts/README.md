# DASPi — four-camera extrinsic orientation solver

This is a **candidate-only** workflow for the `calib4_002` session. It computes yaw/pitch/roll camera-local corrections without modifying live Raspberry Pi services or installed calibration files.

## Why

Your `calib4_002` pose report contains 15 CAM0–CAM1, 16 CAM0–CAM2, and 16 CAM0–CAM3 shared ChArUco board placements. They connect all four cameras to CAM0. The existing C++ `daspi-calibrate-charuco` program fits intrinsics for every module but only writes module 0/1 relative orientation. This tool uses its already-generated `charuco-pose-report.csv` to fit CAM0-anchored orientation corrections for the other three cameras.

## Installation

Copy both `.py` files to `~/DASPi/software/scripts/` and make them executable:

```bash
chmod +x ~/DASPi/software/scripts/{enable_nominal_rotation_logging,solve_four_camera_extrinsics}.py
```

## Step 1 — emit nominal rotations for the actual four-module topology

The extrinsics need **nominal camera orientations from the same current DASPi build**, not the earlier two-camera nominal rotation. The nominal angles from an old two-camera build must **not** be reused for this calibration.

Apply the small, guarded logging patch:

```bash
python3 ~/DASPi/software/scripts/enable_nominal_rotation_logging.py \
  ~/DASPi/software/src/computemodule/src/main.cpp
meson compile -C ~/DASPi/software/build/computemodule-native
```

The helper backs up `main.cpp` as `main.cpp.before-nominal4.bak`. The modification changes only the existing nominal relative-rotation logging function; it does not change geometry.

Restart **computemodule on the laptop with your existing command** (4 modules, same IP mapping), capturing stdout/stderr to a fresh file, e.g.:

```bash
# Add this at the end of your existing computemodule command:
#   2>&1 | tee /tmp/daspi-nominal-4cam.log
```

Once all three `[nominal relative]` lines have appeared, the calibration tool has everything it needs. The program may be stopped normally if you only ran it to retrieve those nominal values. Check:

```bash
grep 'module0_to_module.*_rvec_deg=' /tmp/daspi-nominal-4cam.log
```

Expected: three lines for module1, module2, and module3. **Do not proceed if any are missing.**

## Step 2 — solve with the existing `calib4_002` pose estimates

```bash
D="$HOME/DASPi/software/config/charuco-captures/calib4_002"
python3 ~/DASPi/software/scripts/solve_four_camera_extrinsics.py \
  --pose-csv="$D/charuco-pose-report.csv" \
  --nominal-log=/tmp/daspi-nominal-4cam.log \
  --output="$D/camera-calibration-generated-4cam.txt" \
  --report="$D/extrinsics-quality-report.txt"
```

The candidate file contains four lines, `module yaw pitch roll`, readable by DASPi's current calibration loader. The quality report lists the per-pair angular dispersion, rejected poses, and independent cycle checks for CAM1–CAM2, CAM1–CAM3 and CAM2–CAM3. This uses **3D rotation only**; it does not solve translations, camera positions or a full bundle adjustment.

If the output path exists, the solver refuses to overwrite it. Rename/move the old *candidate* explicitly before rerunning.

## Validate before deployment

- Verify camera module IDs in the output correspond to physical hosts `10.0.2.4` ... `10.0.2.7`. The prior two-camera script used a swapped mapping; the current four-camera system should use `.4` for module 0 through `.7` for module 3.
- Check the `extrinsics-quality-report.txt` for unusually large angular corrections, cross-link disagreement, and high per-pair residuals. A small reprojection RMS does **not** prove the physical rig's nominal mounting geometry is correct.
- Confirm the intrinsics YAML files correspond to the **same `calib4_002` data**, active resolution (1456 × 1088), and undistorted/unaltered sensor pixel coordinates used at runtime.
- Review seam alignment and overlap coverage with a copy of the candidate calibration before changing production calibration on the laptop or any Pi.
- **No deployment is included** in these scripts.
