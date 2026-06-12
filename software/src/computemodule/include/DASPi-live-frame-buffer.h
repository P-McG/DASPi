// DASPi-live-frame-buffer.h
#pragma once
#include <mutex>
#include <cstdint>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

struct LiveFrameBuffer {
    std::mutex mutex;
    cv::Mat latestBgr8;
    std::uint64_t generation{0};
    bool hasFrame{false};
};
