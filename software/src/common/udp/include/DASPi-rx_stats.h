// DASPi-rx_stats.h
#pragma once
#include <chrono>
#include <cstdint>

struct RxStats {
    uint64_t rxComplete = 0;
    uint64_t rxDropOld = 0;
    uint64_t rxTimeout = 0;
    uint64_t rxFail = 0;

    uint64_t lastRxComplete = 0;
    uint64_t lastRxDropOld = 0;
    uint64_t lastRxTimeout = 0;
    uint64_t lastRxFail = 0;

    std::chrono::steady_clock::time_point lastPrint =
        std::chrono::steady_clock::now();
};
