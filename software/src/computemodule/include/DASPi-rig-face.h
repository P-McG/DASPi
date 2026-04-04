// DASPi-rig-face.h
#pragma once

#include <array>

namespace {
/*
 * =========================
 * Generic rig geometry layer
 * =========================
 *
 * Keep geometry-specific math here. The rest of main.cpp should only care
 * about a list of look directions and how they map to modules.
 */

struct RigFace
{
    std::array<int, 3> vi;
    Eigen::Vector3d lookDir;
    Eigen::Vector3d center;
};
};
