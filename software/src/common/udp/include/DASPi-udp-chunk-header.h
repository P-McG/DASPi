// DASPi-udp-chunk-header.h
#pragma once
#include <cstdint>
#include "DASPi-frameheader.h"

namespace DASPi {

#pragma pack(push, 1)
struct UdpChunkHeader
{
    uint32_t magic_;
    uint32_t frameId_;
    uint16_t chunkId_;
    uint16_t chunkCount_;
    uint16_t payloadBytes_;
    uint16_t reserved_;
    FrameHeader frameHeader_;   // valid only for chunkId_ == 0
};
#pragma pack(pop)

} // namespace DASPi
