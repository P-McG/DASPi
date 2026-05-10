// DASPi-spherical-math.h
#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "DASPi-rig-face.h"

namespace DASPi::spherical {

// Equirectangular pixel -> unit world ray
//inline Eigen::Vector3d EquirectPixelToWorldRay(
    //int x, int y, int width, int height)
//{
    //const double u = (static_cast<double>(x) + 0.5) / static_cast<double>(width);
    //const double v = (static_cast<double>(y) + 0.5) / static_cast<double>(height);

    //const double lambda = 2.0 * M_PI * u - M_PI;   // longitude
    //const double phi    = M_PI * (0.5 - v);        // latitude

    //const double cosPhi = std::cos(phi);

    //return Eigen::Vector3d(
        //cosPhi * std::sin(lambda),
        //std::sin(phi),
        //cosPhi * std::cos(lambda)
    //); // already unit length
//}

Eigen::Vector3d EquirectPixelToRay(int x, int y, int width, int height);

// World ray -> camera ray
Eigen::Vector3d WorldToCameraRay(
    const Eigen::Vector3d& ray_world,
    const Eigen::Matrix3d& Rcw);

// Camera ray -> world ray
Eigen::Vector3d CameraToWorldRay(
    const Eigen::Vector3d& ray_cam,
    const Eigen::Matrix3d& Rcw);
    
template<size_t N>
bool IsRayInsideSphericalFace(const Eigen::Vector3d& ray,
                              const RigFace<N>& face,
                              const std::vector<Eigen::Vector3d>& vertices);

} // namespace DASPi::spherical
#include "DASPi-spherical-math.tpp"
