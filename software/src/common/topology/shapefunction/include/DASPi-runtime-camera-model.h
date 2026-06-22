#pragma once

#include <cstdint>
#include <cmath>
#include <numbers>

#include <Eigen/Dense>

#include "DASPi-globallinearspace.h"

namespace DASPi {

/*
 * Runtime projection model shared by:
 *
 *   - ModuleSphericalMap
 *   - daspi-calibrate-charuco
 *
 * This intentionally does NOT include DASPi-icosahedronspace.h because that
 * header is not standalone. The focal length formula below matches
 * DASPi::detail::IcosahedronTables::cameraFocalPx_:
 *
 *   geometricPixelsPerNormalToNormalAngle = sensorHeight * sqrt(3) / 3
 *   pixelsPerNormalToNormalAngle = geometric / lensFovScale
 *   cameraFocalPx = pixelsPerNormalToNormalAngle / normalToNormalAngle
 */
struct RuntimeCameraIntrinsics {
    static constexpr std::uint32_t width{
        static_cast<std::uint32_t>(sensorWidthValue_)
    };

    static constexpr std::uint32_t height{
        static_cast<std::uint32_t>(sensorHeightValue_)
    };

    static constexpr double normalToNormalAngle{
        0.72972765622696635559
    };

    static constexpr double lensFovScale{
        1.2
    };

    static constexpr double geometricPixelsPerNormalToNormalAngle{
        static_cast<double>(sensorHeightValue_) *
        std::numbers::sqrt3 / 3.0
    };

    static constexpr double pixelsPerNormalToNormalAngle{
        geometricPixelsPerNormalToNormalAngle / lensFovScale
    };

    static constexpr double fx{
        pixelsPerNormalToNormalAngle / normalToNormalAngle
    };

    static constexpr double fy{
        fx
    };

    static constexpr double cx{
        static_cast<double>(sensorWidthValue_) * 0.5
    };

    static constexpr double cy{
        static_cast<double>(sensorHeightValue_) * 0.5
    };
};

inline Eigen::Vector3d RuntimePixelToCameraRay(double x, double y)
{
    const double nx =
        (x - RuntimeCameraIntrinsics::cx) / RuntimeCameraIntrinsics::fx;

    const double ny =
        (y - RuntimeCameraIntrinsics::cy) / RuntimeCameraIntrinsics::fy;

    return Eigen::Vector3d(nx, ny, 1.0).normalized();
}

} // namespace DASPi
