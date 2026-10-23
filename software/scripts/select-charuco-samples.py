#!/usr/bin/env python3
"""Select one good native-pixel ChArUco frame per stationary board pose and camera.

The DASPi C++ calibrator otherwise treats every frame from one static placement
as an independent view. This preprocessor balances poses and prints an overlap
matrix for planning inter-camera extrinsics.
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
import re

HEADER = ["session_id", "pose_id", "timestamp_ns", "frame", "module", "facet", "corner_id", "x", "y"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--capture-dir", required=True, type=Path)
    p.add_argument("--session", required=True)
    p.add_argument("--min-corners", default=20, type=int)
    args = p.parse_args()
    folder = args.capture_dir
    pattern = re.compile(rf"^{re.escape(args.session)}-pose([0-9]+)-module([0-3])\.csv$")

    # Each accepted camera/pose can contain several repeated frames.
    frames = defaultdict(dict)
    source_files = 0
    for file in sorted(folder.glob(f"{args.session}-pose*-module*.csv")):
        match = pattern.fullmatch(file.name)
        if not match:
            continue
        expected_pose, expected_module = map(int, match.groups())
        source_files += 1
        with file.open(newline="") as f:
            lines = (line for line in f if not line.lstrip().startswith("#"))
            for row in csv.DictReader(lines):
                if set(HEADER) - set(row):
                    raise ValueError(f"Missing columns in {file}")
                if row["session_id"] != args.session or int(row["pose_id"]) != expected_pose or int(row["module"]) != expected_module:
                    raise ValueError(f"Unexpected session, pose or module ID in {file}")
                frame_key = (expected_pose, expected_module, int(row["frame"]))
                frames[frame_key][int(row["corner_id"])] = row  # deduplicate IDs

    # Keep the frame with the most corners for each pose/module.
    best = {}
    for (pose, mod, frame), corners in frames.items():
        if len(corners) < args.min_corners:
            continue
        key = (pose, mod)
        if key not in best or len(corners) > len(best[key][1]):
            best[key] = (frame, corners)

    poses_by_module = {m: set() for m in range(4)}
    for (pose, mod) in best:
        poses_by_module[mod].add(pose)

    for mod in range(4):
        target = folder / f"daspi-charuco-observations-module{mod}.csv"
        with target.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=HEADER)
            writer.writeheader()
            for (pose, m), (_frame, corners) in sorted(best.items()):
                if m == mod:
                    writer.writerows(corners[k] for k in sorted(corners))
        print(f"CAM {mod}: {len(poses_by_module[mod]):3d} distinct views → {target.name}")

    print(f"Source CSVs: {source_files}; usable camera/pose combinations: {len(best)}")
    print("Shared stationary board poses between camera pairs:")
    for i in range(4):
        for j in range(i+1, 4):
            n = len(poses_by_module[i] & poses_by_module[j])
            print(f"  CAM{i}–CAM{j}: {n}")

    connected = {0}
    while True:
        expanded = connected | {j for i in connected for j in range(4)
                                if poses_by_module[i] & poses_by_module[j]}
        if expanded == connected:
            break
        connected = expanded
    if len(connected) == 4:
        print("Overlap graph: connected (all cameras can be related through shared poses).")
    else:
        print(f"Overlap graph: INCOMPLETE; cameras reachable from CAM0 = {sorted(connected)}")
        print("Relative orientation of disconnected cameras needs shared board poses or another reference.")

    fewer = [m for m in range(4) if len(poses_by_module[m]) < 10]
    if fewer:
        print(f"Intrinsics not yet ready for modules {fewer}: solver requires ≥10 usable views each.")
        print("Collect more distinct poses before running the intrinsics solver.")
    else:
        print("Intrinsics: all four have the solver's minimum 10 usable views;")
        print("15–25 varied poses per camera is a stronger practical target.")


if __name__ == "__main__":
    main()
