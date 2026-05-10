// DASPi-sphere-stitch-config.h
#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "DASPi-stitch_mode.h"

namespace DASPi{
struct SphereStitchConfig {
    int outputWidth = 4096;
    int outputHeight = 2048;
    double blendPower = 4.0;
    cv::Vec3b backgroundColor = {0, 0, 0};
    
    StitchMode mode = StitchMode::Blend;
};
}//DASPi
