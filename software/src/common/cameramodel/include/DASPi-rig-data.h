// DASPi-rig-data.h
#pragma once

#include "DASPi-rig-face.h"

template <std::size_t N>
struct RigData {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<RigFace<N>> faces;
};
