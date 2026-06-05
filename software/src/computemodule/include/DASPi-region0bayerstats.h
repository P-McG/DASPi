// DASPi-region0bayerstats.h
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>

namespace DASPi::detail{

struct Region0BayerStats {
    double rSum01 = 0.0;
    double gSum01 = 0.0;
    double bSum01 = 0.0;

    std::size_t rCount = 0;
    std::size_t gCount = 0;
    std::size_t bCount = 0;

    void addRed(uint16_t v)
    {
        rSum01 += static_cast<double>(v) / 65535.0;
        ++rCount;
    }

    void addGreen(uint16_t v)
    {
        gSum01 += static_cast<double>(v) / 65535.0;
        ++gCount;
    }

    void addBlue(uint16_t v)
    {
        bSum01 += static_cast<double>(v) / 65535.0;
        ++bCount;
    }

    [[nodiscard]]
    bool hasWhiteBalanceSamples() const noexcept
    {
        return rCount != 0 && gCount != 0 && bCount != 0;
    }

    [[nodiscard]]
    float meanBrightness01() const noexcept
    {
        const std::size_t totalCount = rCount + gCount + bCount;

        if (totalCount == 0) {
            return 0.0f;
        }

        const double totalSum = rSum01 + gSum01 + bSum01;

        return static_cast<float>(
            totalSum / static_cast<double>(totalCount)
        );
    }
};
}//DASPi
