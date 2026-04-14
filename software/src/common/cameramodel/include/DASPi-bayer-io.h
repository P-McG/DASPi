// DASPi-bayer-io.h
#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace DASPi {

cv::Mat LoadBayer16AsBgr8(const std::string& path,
                          int width,
                          int height,
                          int bayerCode = cv::COLOR_BayerBG2BGR);

}
