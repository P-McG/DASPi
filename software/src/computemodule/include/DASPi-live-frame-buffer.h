// DASPi-live-frame-buffer.h
#pragma once
#include <mutex>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

struct LiveFrameBuffer {
    std::mutex mutex;
    cv::Mat latestBgr8;
    bool hasFrame{false};
};
