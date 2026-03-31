// DASPi-rx-frame-assembly.h
#pragma once
#include "DASPi-framepacket.h"

namespace DASPi{
struct RxFrameAssembly
{
    FrameHeader frameHeader{};
    bool headerSeen{false};

    uint32_t frameId{0};
    uint16_t chunkCount{0};
    size_t totalBytesExpected{0};
    size_t totalBytesReceived{0};

    std::vector<uint16_t> payload;
    std::vector<uint8_t> chunkSeen;
};
}//namespace DASPi
