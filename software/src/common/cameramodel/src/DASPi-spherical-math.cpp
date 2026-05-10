// DASPi-spherical-math.cpp

#include <Eigen/Dense>
#include <cmath>

#include "DASPi-spherical-math.h"

namespace DASPi::spherical {

Eigen::Vector3d EquirectPixelToRay(int x, int y, int width, int height)
{
    const double u = (x + 0.5) / static_cast<double>(width);
    const double v = (y + 0.5) / static_cast<double>(height);

    const double theta = (u - 0.5) * 2.0 * M_PI;   // longitude
    const double phi   = (0.5 - v) * M_PI;         // latitude

    const double cosPhi = std::cos(phi);

    return Eigen::Vector3d(
        cosPhi * std::sin(theta),  // X
        std::sin(phi),             // Y
        cosPhi * std::cos(theta)   // Z
    );
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
