// DASPi-contribution.h
#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace DASPi{
struct Contribution {
    int cameraIndex = -1;
    cv::Point2d uv;
    cv::Vec3d color;
    double weight = 0.0;
    bool fromNonOverlap = false;
};
}//DASPi
