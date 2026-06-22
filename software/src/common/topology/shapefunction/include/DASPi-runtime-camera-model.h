#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include <Eigen/Dense>

#include "DASPi-globallinearspace.h"

namespace DASPi {

/*
 * Shared pixel -> camera-ray model.
 *
 * The default values preserve the old runtime gnomonic model, but the fields
 * can be overwritten with calibrated OpenCV intrinsics/distortion from
 * camera-intrinsics-moduleN.yml.
 */
struct RuntimeCameraIntrinsics {
    static constexpr std::uint32_t defaultWidth{
        static_cast<std::uint32_t>(sensorWidthValue_)
    };

    static constexpr std::uint32_t defaultHeight{
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

    static constexpr double defaultFx{
        pixelsPerNormalToNormalAngle / normalToNormalAngle
    };

    static constexpr double defaultFy{
        defaultFx
    };

    static constexpr double defaultCx{
        static_cast<double>(sensorWidthValue_) * 0.5
    };

    static constexpr double defaultCy{
        static_cast<double>(sensorHeightValue_) * 0.5
    };

    std::uint32_t width{defaultWidth};
    std::uint32_t height{defaultHeight};

    double fx{defaultFx};
    double fy{defaultFy};
    double cx{defaultCx};
    double cy{defaultCy};

    /*
     * OpenCV 5-coefficient Brown-Conrady model:
     *   k1, k2, p1, p2, k3
     */
    std::array<double, 5> dist{{0.0, 0.0, 0.0, 0.0, 0.0}};
};

inline Eigen::Vector2d RuntimeUndistortNormalized(
    double xd,
    double yd,
    const RuntimeCameraIntrinsics& intrinsics)
{
    double x = xd;
    double y = yd;

    const double k1 = intrinsics.dist[0];
    const double k2 = intrinsics.dist[1];
    const double p1 = intrinsics.dist[2];
    const double p2 = intrinsics.dist[3];
    const double k3 = intrinsics.dist[4];

    for (int iter = 0; iter < 8; ++iter) {
        const double x2 = x * x;
        const double y2 = y * y;
        const double xy = x * y;
        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double radial =
            1.0 + k1 * r2 + k2 * r4 + k3 * r6;

        if (std::abs(radial) < 1.0e-12) {
            break;
        }

        const double tangentialX =
            2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);

        const double tangentialY =
            p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy;

        x = (xd - tangentialX) / radial;
        y = (yd - tangentialY) / radial;
    }

    return Eigen::Vector2d(x, y);
}

inline Eigen::Vector3d RuntimePixelToCameraRay(
    double pixelX,
    double pixelY,
    const RuntimeCameraIntrinsics& intrinsics)
{
    const double xd =
        (pixelX - intrinsics.cx) / intrinsics.fx;

    const double yd =
        (pixelY - intrinsics.cy) / intrinsics.fy;

    const Eigen::Vector2d undistorted =
        RuntimeUndistortNormalized(xd, yd, intrinsics);

    return Eigen::Vector3d(
        undistorted.x(),
        undistorted.y(),
        1.0
    ).normalized();
}

inline Eigen::Vector3d RuntimePixelToCameraRay(double pixelX, double pixelY)
{
    return RuntimePixelToCameraRay(
        pixelX,
        pixelY,
        RuntimeCameraIntrinsics{}
    );
}

} // namespace DASPi
