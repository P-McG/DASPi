// DASPi-spherical-math.cpp

#include <Eigen/Dense>
#include <cmath>

#include "DASPi-spherical-math.h"

namespace spherical {

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

bool IsRayInsideSphericalFace(const Eigen::Vector3d& ray,
                              const RigFace& face,
                              const std::vector<Eigen::Vector3d>& vertices)
{
    const Eigen::Vector3d r = ray.normalized();

    const Eigen::Vector3d a =
        vertices[static_cast<size_t>(face.vi[0])].normalized();
    const Eigen::Vector3d b =
        vertices[static_cast<size_t>(face.vi[1])].normalized();
    const Eigen::Vector3d c =
        vertices[static_cast<size_t>(face.vi[2])].normalized();

    const double d0 = a.cross(b).dot(r);
    const double d1 = b.cross(c).dot(r);
    const double d2 = c.cross(a).dot(r);

    const bool allPos = (d0 >= 0.0) && (d1 >= 0.0) && (d2 >= 0.0);
    const bool allNeg = (d0 <= 0.0) && (d1 <= 0.0) && (d2 <= 0.0);

    return allPos || allNeg;
}

} // namespace spherical
