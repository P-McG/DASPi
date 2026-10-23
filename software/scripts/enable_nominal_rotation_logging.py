#!/usr/bin/env python3
"""Patch computemodule/src/main.cpp to print nominal CAM0->CAM1/2/3 rotations.

Only edits the existing diagnostics function. Creates a side-by-side backup.
"""
from pathlib import Path
import shutil
import sys

if len(sys.argv)!=2:
    sys.exit("Usage: enable_nominal_rotation_logging.py PATH/TO/software/src/computemodule/src/main.cpp")

path=Path(sys.argv[1])
source=path.read_text()
if "module0_to_module3_rvec_deg" in source:
    sys.exit("Module 3 nominal rotation printing appears already present; no modification made")
needle="void PrintNominalModule0ToModule1RvecDeg("
if source.count(needle)!=1:
    sys.exit("Could not identify exactly one nominal-rotation diagnostics function; not modifying source")
start=source.index(needle)
end=source.index("\n} // namespace",start)
old=source[start:end]
if old.count("module0_to_module1_rvec_deg")!=1 or "FindMainModuleCameraConfig" not in old:
    sys.exit("Existing function does not match expected DASPi structure; not modifying source")

replacement=r'''void PrintNominalModule0ToModule1RvecDeg(
    const std::vector<CameraConfig>& configs)
{
    const CameraConfig* module0 =
        FindMainModuleCameraConfig(configs, 0);

    if (module0 == nullptr) {
        std::cerr << "[nominal relative] unable to find module 0 main camera config\n";
        return;
    }

    // Rcw maps camera rays to world rays. For CAM0 -> CAMi:
    // R_i0 = Rcw_i^T * Rcw_0.
    for (int module = 1; module <= 3; ++module) {
        const CameraConfig* other =
            FindMainModuleCameraConfig(configs, module);

        if (other == nullptr) {
            continue;
        }

        const Eigen::Matrix3d R_i0 =
            other->Rcw.transpose() * module0->Rcw;

        const Eigen::AngleAxisd aa(R_i0);
        const double pi = 3.141592653589793238462643383279502884;
        const Eigen::Vector3d rvecDeg =
            aa.axis() * aa.angle() * (180.0 / pi);

        std::cout
            << std::fixed << std::setprecision(9)
            << "[nominal relative] module0_to_module"
            << module << "_rvec_deg="
            << rvecDeg.x() << ',' << rvecDeg.y() << ',' << rvecDeg.z()
            << '\n';
        std::cout
            << "[nominal relative] module0_config=" << module0->name
            << " module" << module << "_config=" << other->name
            << '\n';
    }
}
'''
backup=path.with_name(path.name+".before-nominal4.bak")
if backup.exists():
    sys.exit(f"Backup file already exists, not overwriting: {backup}")
shutil.copy2(path,backup)
path.write_text(source[:start]+replacement+source[end:])
print(f"Updated {path}")
print(f"Saved original to {backup}")
