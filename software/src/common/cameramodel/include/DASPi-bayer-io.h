// DASPi-bayer-io.h
#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace DASPi {

cv::Mat decodeBayer16ToBgr8(const std::uint16_t* data,
                            int width,
                            int height,
                            int bayerCode = cv::COLOR_BayerBG2BGR);

}
