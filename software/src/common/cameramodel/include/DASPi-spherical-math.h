// DASPi-spherical-math.h
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace spherical {

// Equirectangular pixel -> unit world ray
inline Eigen::Vector3d EquirectPixelToWorldRay(
    int x, int y, int width, int height)
{
    const double u = (static_cast<double>(x) + 0.5) / static_cast<double>(width);
    const double v = (static_cast<double>(y) + 0.5) / static_cast<double>(height);

    const double lambda = 2.0 * M_PI * u - M_PI;   // longitude
    const double phi    = M_PI * (0.5 - v);        // latitude

    const double cosPhi = std::cos(phi);

    return Eigen::Vector3d(
        cosPhi * std::sin(lambda),
        std::sin(phi),
        cosPhi * std::cos(lambda)
    ); // already unit length
}

// World ray -> camera ray
inline Eigen::Vector3d WorldToCameraRay(
    const Eigen::Vector3d& ray_world,
    const Eigen::Matrix3d& Rcw)
{
    return Rcw.transpose() * ray_world;
}

// Camera ray -> world ray
inline Eigen::Vector3d CameraToWorldRay(
    const Eigen::Vector3d& ray_cam,
    const Eigen::Matrix3d& Rcw)
{
    return Rcw * ray_cam;
}

} // namespace spherical
