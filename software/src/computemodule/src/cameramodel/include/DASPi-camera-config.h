// DASPi-camera-config.h
#pragma once
#include <string>
#include "DASPi-image-rotation.h"

struct CameraConfig {
    std::string name;
    std::string path;
    ImageRotation imageRotation;
    Eigen::Matrix3d Rcw;
};
