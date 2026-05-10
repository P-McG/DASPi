// DASPi-flat_triangle_map.h
#pragma once
#include <array>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace DASPi{
struct FlatTriangleMap {
    std::array<cv::Point2d, 3> p;
};
}//DASPi
