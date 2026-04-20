// DASPi-rig-face.h
#pragma once

#include <array>

/*
 * =========================
 * Generic rig geometry layer
 * =========================
 *
 * Keep geometry-specific math here. The rest of main.cpp should only care
 * about a list of look directions and how they map to modules.
 */

template <std::size_t N>
struct RigFace {
    std::array<std::size_t, N> indices;
    Eigen::Vector3d normal;
    Eigen::Vector3d lookDir;
};
