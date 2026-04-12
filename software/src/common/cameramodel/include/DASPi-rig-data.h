// DASPi-rig-data.h
#pragma once

#include "DASPi-rig-face.h"

struct RigData {
    std::vector<RigFace> faces;
    std::vector<Eigen::Vector3d> vertices;
};
