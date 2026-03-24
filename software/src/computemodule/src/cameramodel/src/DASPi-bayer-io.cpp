// DASPi-bayer-io.cpp
#include "DASPi-bayer-io.h"

#include <fstream>
#include <stdexcept>

namespace DASPi {

cv::Mat LoadBayer16AsBgr8(const std::string& path,
                          int width,
                          int height,
                          int bayerCode)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open Bayer file: " + path);
    }

    cv::Mat raw16(height, width, CV_16UC1);
    file.read(reinterpret_cast<char*>(raw16.data),
              static_cast<std::streamsize>(width) *
              static_cast<std::streamsize>(height) *
              static_cast<std::streamsize>(sizeof(uint16_t)));

    if (!file) {
        throw std::runtime_error("Failed to read full Bayer frame: " + path);
    }

    cv::Mat bgr16;
    cv::cvtColor(raw16, bgr16, bayerCode);

    cv::Mat bgr8;
    bgr16.convertTo(bgr8, CV_8UC3, 1.0 / 256.0);

    return bgr8;
}

}
