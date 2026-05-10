// DASPi-spherical-math.cpp

#include <Eigen/Dense>
#include <cmath>

#include "DASPi-spherical-math.h"

namespace DASPi::spherical {
    
template <std::size_t N>
bool IsRayInsideSphericalFace(const Eigen::Vector3d& ray,
                             const RigFace<N>& face,
                             const std::vector<Eigen::Vector3d>& vertices)
{
    constexpr double kEps = 1e-12;

    for (std::size_t i = 0; i < N; ++i) {
        const std::size_t j = (i + 1) % N;

        const Eigen::Vector3d& a = vertices[face.indices[i]];
        const Eigen::Vector3d& b = vertices[face.indices[j]];

        Eigen::Vector3d edgePlaneNormal = a.cross(b);
        if (edgePlaneNormal.norm() == 0.0) {
            return false;
        }
        edgePlaneNormal.normalize();

        if (edgePlaneNormal.dot(face.lookDir) < 0.0) {
            edgePlaneNormal = -edgePlaneNormal;
        }

        if (edgePlaneNormal.dot(ray) < -kEps) {
            return false;
        }
    }

    return true;
}

} // namespace spherical
