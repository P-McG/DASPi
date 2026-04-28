// DASPi-camera_setup.h
#pragma once
#include <vector>
#include "DASPi-camera-config.h"
#include "DASPi-rig-data.h"

template <std::size_t N>
struct CameraSetup {
    std::vector<CameraConfig> configs;
    RigData<N> rig;
};
