// DASPi-camera-config.h
#pragma once
#include <string>
#include "DASPi-image-rotation.h"

struct CameraConfig {
    std::string name;
    std::string sourceName;
    ImageRotation imageRotation;
    Eigen::Matrix3d Rcw;
    int moduleIndex = -1;
    int faceIndex = -1;
};

//struct CameraConfig {
    //std::string name;
    //std::string path;
    //ImageRotation imageRotation;
    //Eigen::Matrix3d Rcw;
//};
