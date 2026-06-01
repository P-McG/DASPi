// DASPi-region0whitebalancegains.h
#pragma once

namespace DASPi::detail{
struct Region0WhiteBalanceGains {
    float rGainApply = 1.0f;
    float bGainApply = 1.0f;
    std::size_t contributingRegion0s = 0;
    bool valid = false;
};
}//DASPi
