// DASPi-camera-view.h
#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-i-camera-model.h"

struct CameraView {
    cv::Mat image;
    cv::Mat sensorValidMask;   // optional; only for true sensor/image validity
    std::shared_ptr<ICameraModel> model;
    Eigen::Matrix3d Rcw;

    int moduleIndex = -1;
    int faceIndex = -1;
};
