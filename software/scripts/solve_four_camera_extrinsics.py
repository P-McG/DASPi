#!/usr/bin/env python3
"""Compute CAM0-anchored, four-camera orientation corrections from DASPi PnP pose CSV.

Pure Python 3, no third-party dependencies. Produces only a *candidate* file.
"""
import argparse
import csv
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path

PI = math.pi


def unit(q):
    n = math.sqrt(sum(x*x for x in q))
    if n < 1e-15:
        raise ValueError("Zero quaternion")
    return tuple(x/n for x in q)


def conj(q):
    return q[0], -q[1], -q[2], -q[3]


def mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw-ax*bx-ay*by-az*bz,
            aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw)


def rotvec_to_q(vector):
    theta = math.sqrt(sum(x*x for x in vector))
    if theta < 1e-13:
        return unit((1.0, vector[0]/2, vector[1]/2, vector[2]/2))
    s = math.sin(theta/2)/theta
    return unit((math.cos(theta/2), *(x*s for x in vector)))


def q_to_matrix(q):
    w,x,y,z = unit(q)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def qdist(a, b):
    d = abs(sum(x*y for x,y in zip(unit(a), unit(b))))
    return math.degrees(2*math.acos(max(-1.0, min(1.0, d))))


def yaw_pitch_roll(q):
    # Inverse of DASPi: local Ry(yaw) * Rx(pitch) * Rz(roll).
    R = q_to_matrix(q)
    pitch = math.asin(max(-1., min(1., -R[1][2])))
    if abs(math.cos(pitch)) > 1e-9:
        yaw = math.atan2(R[0][2], R[2][2])
        roll = math.atan2(R[1][0], R[1][1])
    else:
        yaw = math.atan2(-R[2][0], R[0][0])
        roll = 0.
    return tuple(math.degrees(a) for a in (yaw,pitch,roll))


def fit_quats(views):
    """Weighted quaternion medoid -> robust mean, return q, diagnostics."""
    # views: (q,weight,pose_id)
    if not views:
        raise ValueError("No shared observations")
    # Medoid resists severe outliers better than averaging first.
    medoid = min(views, key=lambda v: sum(w*qdist(v[0], p) for p,w,_ in views))[0]
    distances = [qdist(q, medoid) for q,_,_ in views]
    middle = statistics.median(distances)
    mad = statistics.median(abs(d-middle) for d in distances)
    gate = max(2.5, middle + 3*1.4826*mad)
    accepted = [v for v, d in zip(views, distances) if d <= gate]
    if len(accepted) < 2:
        accepted = views[:]
    for _ in range(3):
        anchor = medoid
        total = [0.,0.,0.,0.]
        for q, w, _ in accepted:
            sign = 1 if sum(x*y for x,y in zip(q,anchor)) >= 0 else -1
            for k in range(4): total[k] += sign*w*q[k]
        medoid = unit(total)
    residuals = [qdist(q,medoid) for q,_,_ in views]
    good = [qdist(q,medoid) for q,_,_ in accepted]
    return medoid, {
        "total":len(views), "used":len(accepted), "median_deg":statistics.median(good),
        "max_deg":max(good), "rejected":len(views)-len(accepted),
        "all_max_deg":max(residuals),
        "poses": [str(pose) for _,_,pose in accepted],
    }


def parse_pose_csv(path):
    poses = defaultdict(dict)
    with path.open(newline="") as f:
        reader=csv.DictReader(f)
        for row in reader:
            try:
                k=(row["session_id"],int(row["pose_id"]))
                cam=int(row["module"])
                if cam not in range(4): continue
                rms=float(row["rms_px"])
                if not math.isfinite(rms) or rms < 0: continue
                rv=tuple(float(row[f"rvec_{axis}"]) for axis in "xyz")
                if not all(math.isfinite(x) for x in rv): continue
                item=(rotvec_to_q(rv),rms,int(row["corners"]),row.get("timestamp_ns",""))
                # If multiple frames per pose, keep lowest RMS frame.
                if cam not in poses[k] or rms < poses[k][cam][1]:
                    poses[k][cam]=item
            except (ValueError,KeyError) as e:
                raise ValueError(f"Malformed pose CSV row: {row}") from e
    return poses


def parse_nominals(log):
    result={}
    pattern=re.compile(r"module0_to_module([123])_rvec_deg=\s*([+\-0-9.eE]+)\s*,\s*([+\-0-9.eE]+)\s*,\s*([+\-0-9.eE]+)")
    for match in pattern.finditer(log.read_text(errors="replace")):
        cam=int(match.group(1))
        angles=[math.radians(float(match.group(i))) for i in (2,3,4)]
        result[cam]=rotvec_to_q(angles)
    if set(result)!={1,2,3}:
        raise ValueError("Nominal log must contain module0_to_module1/2/3_rvec_deg. "
                         f"Found: {sorted(result)}. Apply the nominal-log patch and rerun computemodule.")
    return {0:(1.,0.,0.,0.),**result}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-csv",type=Path,required=True)
    parser.add_argument("--nominal-log",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--report",type=Path,required=True)
    parser.add_argument("--max-warning-deg",type=float,default=5.0)
    args=parser.parse_args()
    poses=parse_pose_csv(args.pose_csv)
    nom=parse_nominals(args.nominal_log)
    observations={}
    fitted={}
    report=[]
    report.append("DASPi four-camera orientation calibration (CAM0 reference)")
    report.append(f"Source PnP: {args.pose_csv}")
    report.append(f"Source nominal: {args.nominal_log}")
    report.append(f"Board placements: {len(poses)}")
    report.append("Pairs: total / accepted / median residual (deg) / max accepted (deg)")
    for a in range(4):
        for b in range(a+1,4):
            views=[]
            for pose_key, cams in poses.items():
                if a not in cams or b not in cams: continue
                qa,rmsa,_,_=cams[a]
                qb,rmsb,_,_=cams[b]
                # board->CAM b times CAM a->board
                qba=unit(mul(qb,conj(qa)))
                weight=1.0/(max(0.5,rmsa+rmsb)**2)
                views.append((qba,weight,pose_key))
            observations[(a,b)]=views
            if views:
                fitted[(a,b)]=fit_quats(views)
                data=fitted[(a,b)][1]
                report.append(f"CAM{a}-CAM{b}: {data['total']:2d} / {data['used']:2d} / "
                              f"{data['median_deg']:.3f} / {data['max_deg']:.3f}; rejected={data['rejected']}")
            else:
                report.append(f"CAM{a}-CAM{b}: no shared board placements")
    if any((0,i) not in fitted or fitted[(0,i)][1]["used"]<3 for i in (1,2,3)):
        raise ValueError("Need at least 3 accepted shared placements directly linking CAM0 to each CAM1..3")
    observed={0:(1.,0.,0.,0.)}
    for i in (1,2,3): observed[i]=fitted[(0,i)][0]
    corrections={0:(1.,0.,0.,0.)}
    for i in (1,2,3):
        corrections[i]=unit(mul(nom[i],conj(observed[i])))
    report.append("\nLoop consistency: direct CAM0 links vs independent CAMi-CAMj estimates")
    loop_errors=[]
    for a in (1,2,3):
        for b in range(a+1,4):
            if (a,b) not in fitted: continue
            predicted=unit(mul(observed[b],conj(observed[a])))
            e=qdist(predicted,fitted[(a,b)][0])
            loop_errors.append(e)
            report.append(f"CAM{a}-CAM{b}: difference {e:.3f} degrees")
    report.append("\nProposed local yaw pitch roll corrections (degrees):")
    warnings=[]
    for i in range(4):
        yaw,pitch,roll=yaw_pitch_roll(corrections[i])
        magnitude=qdist(corrections[i],(1.,0.,0.,0.))
        report.append(f"CAM{i}: yaw={yaw:+.6f} pitch={pitch:+.6f} roll={roll:+.6f} "
                      f"total_angle={magnitude:.3f}")
        if magnitude>20:
            warnings.append(f"CAM{i}: large correction ({magnitude:.2f} degrees), verify nominal mapping/rotation")
    for i in (1,2,3):
        med=fitted[(0,i)][1]["median_deg"]
        if med>args.max_warning_deg:
            warnings.append(f"CAM0-CAM{i}: high residual median {med:.2f} degrees")
    for e in loop_errors:
        if e>args.max_warning_deg:
            warnings.append(f"Loop inconsistency of {e:.2f} degrees exceeds {args.max_warning_deg:.1f} degrees")
    if warnings:
        report.extend(["", "WARNINGS — DO NOT DEPLOY UNTIL INVESTIGATED:",*["- "+s for s in warnings]])
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text("\n".join(report)+"\n")
    # Output only candidate file, never overwrite existing installed calibration.
    if args.output.exists():
        raise FileExistsError(f"Will not overwrite existing candidate calibration: {args.output}")
    lines=["# DASPi candidate four-camera mounting corrections",
           "# CAM0 anchored; nominal relative rotations read from computemodule's current topology",
           "# module_index yaw_deg pitch_deg roll_deg",
           f"# pose_csv={args.pose_csv}", f"# nominal_log={args.nominal_log}",
           "# VALIDATE with seam geometry before installing"]
    if warnings:
        lines.append("# WARNING: solver diagnostic report contains issues; DO NOT DEPLOY")
    for i in range(4):
        yaw,pitch,roll=yaw_pitch_roll(corrections[i])
        lines.append(f"{i} {yaw:.9f} {pitch:.9f} {roll:.9f}")
    args.output.write_text("\n".join(lines)+"\n")
    print("\n".join(report))
    print(f"\nCandidate: {args.output}\nReport:    {args.report}")


if __name__=="__main__":
    main()
