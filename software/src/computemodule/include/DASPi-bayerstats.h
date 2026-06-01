// DASPi-bayerstats.h
#pragma once

namespace DASPi{

struct BayerStats {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    std::size_t rCount = 0;
    std::size_t gCount = 0;
    std::size_t bCount = 0;
};
}//DASPi
