// DASPi-camera-view.h
#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-i-camera-model.h"

struct CameraView {
    cv::Mat image;

    cv::Mat maskNonOverlap;
    cv::Mat maskOverlap;

    std::shared_ptr<ICameraModel> model;
    Eigen::Matrix3d Rcw;

    int faceIndex = -1;              // keep if you're using it
    
    int localStreamIndex = -1;
    int localEdgeIndex = -1;
    int neighborFaceIndex = -1;
    int edgeIndex = -1;
    
    cv::Mat sensorValidMask;         // keep if you're using it
};
