// DASPi-fps-counter.h
#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

class FPSCounter {
public:
    explicit FPSCounter(std::string name,
                        std::chrono::milliseconds reportInterval = std::chrono::milliseconds(1000))
        : name_(std::move(name))
        , reportInterval_(reportInterval)
        , windowStart_(Clock::now())
        , lastReport_(windowStart_) {}

    void Tick() {
        ++totalFrames_;
        ++windowFrames_;

        const auto now = Clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReport_);

        if (elapsed >= reportInterval_) {
            const double seconds =
                static_cast<double>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now - windowStart_).count()
                ) / 1'000'000.0;

            const double fps = (seconds > 0.0)
                ? static_cast<double>(windowFrames_) / seconds
                : 0.0;

            std::cout << "[FPS] " << name_
                      << " fps=" << fps
                      << " window_frames=" << windowFrames_
                      << " total_frames=" << totalFrames_
                      << std::endl;

            windowStart_ = now;
            lastReport_  = now;
            windowFrames_ = 0;
        }
    }

    std::uint64_t TotalFrames() const { return totalFrames_; }

private:
    using Clock = std::chrono::steady_clock;

    std::string name_;
    std::chrono::milliseconds reportInterval_;

    Clock::time_point windowStart_;
    Clock::time_point lastReport_;

    std::uint64_t totalFrames_  = 0;
    std::uint64_t windowFrames_ = 0;
};
