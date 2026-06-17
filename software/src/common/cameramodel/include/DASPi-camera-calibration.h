// DASPi-camera-calibration.h
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>

#include <Eigen/Dense>

#include "DASPi-camera-config.h"

namespace DASPi {

struct CameraCalibrationCorrection {
    double yawDeg{0.0};
    double pitchDeg{0.0};
    double rollDeg{0.0};

    bool IsZero() const
    {
        return yawDeg == 0.0 && pitchDeg == 0.0 && rollDeg == 0.0;
    }
};

inline double DegToRad(double deg)
{
    return deg * M_PI / 180.0;
}

/*
 * Local camera-axis correction.
 *
 * Right-multiply this onto Rcw:
 *
 *     RcwCorrected = RcwNominal * RdeltaLocal
 *
 * This keeps the compile-time/global topology as the nominal rig and applies
 * the physical mounting correction in the camera's local coordinate frame.
 */
inline Eigen::Matrix3d MakeCameraCalibrationDeltaLocal(
    const CameraCalibrationCorrection& c)
{
    const Eigen::Matrix3d Ryaw =
        Eigen::AngleAxisd(DegToRad(c.yawDeg),
                          Eigen::Vector3d::UnitY()).toRotationMatrix();

    const Eigen::Matrix3d Rpitch =
        Eigen::AngleAxisd(DegToRad(c.pitchDeg),
                          Eigen::Vector3d::UnitX()).toRotationMatrix();

    const Eigen::Matrix3d Rroll =
        Eigen::AngleAxisd(DegToRad(c.rollDeg),
                          Eigen::Vector3d::UnitZ()).toRotationMatrix();

    /*
     * Local convention:
     *   yaw   = camera local +Y
     *   pitch = camera local +X
     *   roll  = camera local +Z
     */
    return Ryaw * Rpitch * Rroll;
}

inline std::string StripCommentAndTrim(std::string line)
{
    const std::size_t hash = line.find('#');
    if (hash != std::string::npos) {
        line.erase(hash);
    }

    const auto notSpace = [](unsigned char c) {
        return !std::isspace(c);
    };

    line.erase(
        line.begin(),
        std::find_if(line.begin(), line.end(), notSpace)
    );

    line.erase(
        std::find_if(line.rbegin(), line.rend(), notSpace).base(),
        line.end()
    );

    return line;
}

inline std::string ExpandUserPath(const std::string& path)
{
    if (path.empty()) {
        return path;
    }

    if (path == "~") {
        const char* home = std::getenv("HOME");
        if (home == nullptr || std::string(home).empty()) {
            throw std::runtime_error("Cannot expand '~': HOME is not set");
        }
        return std::string(home);
    }

    if (path.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home == nullptr || std::string(home).empty()) {
            throw std::runtime_error("Cannot expand '~/': HOME is not set");
        }

        return std::string(home) + path.substr(1);
    }

    return path;
}

/*
 * Simple calibration file format:
 *
 *     # module yaw_deg pitch_deg roll_deg
 *     0 -0.42  1.18  0.27
 *     1  0.66 -0.91 -0.14
 *
 * Missing file path means all-zero corrections.
 */
inline std::vector<CameraCalibrationCorrection>
LoadCameraCalibrationCorrections(
    const std::string& path,
    std::size_t moduleCount)
{
    std::vector<CameraCalibrationCorrection> corrections(moduleCount);

    if (path.empty()) {
        std::cout << "[camera calibration] no calibration file supplied; "
                  << "using zero corrections\n";
        return corrections;
    }

    const std::string expandedPath = ExpandUserPath(path);

    std::ifstream in(expandedPath);

    if (!in) {
        throw std::runtime_error(
            "Failed to open camera calibration file: " + expandedPath
        );
    }

    std::string line;
    std::size_t lineNumber = 0;

    while (std::getline(in, line)) {
        ++lineNumber;

        line = StripCommentAndTrim(std::move(line));
        if (line.empty()) {
            continue;
        }

        std::istringstream ss(line);

        std::size_t moduleIndex = 0;
        CameraCalibrationCorrection c{};

        if (!(ss >> moduleIndex >> c.yawDeg >> c.pitchDeg >> c.rollDeg)) {
            throw std::runtime_error(
                "Invalid camera calibration line " +
                std::to_string(lineNumber) +
                ". Expected: <module> <yaw_deg> <pitch_deg> <roll_deg>"
            );
        }

        if (moduleIndex >= moduleCount) {
            throw std::runtime_error(
                "Camera calibration module index out of range on line " +
                std::to_string(lineNumber) +
                ": " + std::to_string(moduleIndex)
            );
        }

        corrections[moduleIndex] = c;
    }

    std::cout << "[camera calibration] loaded " << expandedPath << '\n';

    for (std::size_t module = 0; module < corrections.size(); ++module) {
        const auto& c = corrections[module];

        std::cout << "  module[" << module << "]"
                  << " yaw_deg=" << c.yawDeg
                  << " pitch_deg=" << c.pitchDeg
                  << " roll_deg=" << c.rollDeg
                  << '\n';
    }

    return corrections;
}

inline void ApplyCameraCalibrationCorrections(
    std::vector<CameraConfig>& configs,
    const std::vector<CameraCalibrationCorrection>& corrections)
{
    for (CameraConfig& cfg : configs) {
        if (cfg.moduleIndex < 0) {
            continue;
        }

        const std::size_t moduleIndex =
            static_cast<std::size_t>(cfg.moduleIndex);

        if (moduleIndex >= corrections.size()) {
            throw std::runtime_error(
                "CameraConfig moduleIndex out of calibration range: " +
                std::to_string(cfg.moduleIndex)
            );
        }

        const CameraCalibrationCorrection& c =
            corrections[moduleIndex];

        const Eigen::Matrix3d Rdelta =
            MakeCameraCalibrationDeltaLocal(c);

        cfg.Rcw = cfg.Rcw * Rdelta;

        cfg.calibrationYawDeg = c.yawDeg;
        cfg.calibrationPitchDeg = c.pitchDeg;
        cfg.calibrationRollDeg = c.rollDeg;
    }
}

} // namespace DASPi
