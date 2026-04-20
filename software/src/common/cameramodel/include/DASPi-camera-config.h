// DASPi-camera-config.h
#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "DASPi-image-rotation.h"

struct CameraConfig {
    std::string name;
    std::string device = "";
    std::string sourceName;
    ImageRotation imageRotation;
    Eigen::Matrix3d Rcw;
    int moduleIndex = -1;
    cv::Mat sensorValidMask = cv::Mat();  // disables mask check cleanly
    int faceIndex = -1;
};

//struct CameraConfig {
    //std::string name;
    //std::string path;
    //ImageRotation imageRotation;
    //Eigen::Matrix3d Rcw;
//};
