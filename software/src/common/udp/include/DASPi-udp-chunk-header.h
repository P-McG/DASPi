// DASPi-udp-chunk-header.h
#pragma once
#include "DASPi-frameheader.h"

#pragma pack(push, 1)
struct UdpChunkHeader
{
    uint32_t magic_;
    uint32_t frameId_;
    uint16_t chunkId_;
    uint16_t chunkCount_;
    uint16_t payloadBytes_;
    uint16_t reserved_;

    FrameHeader frameHeader_; // valid only when chunkId_ == 0
};
#pragma pack(pop)
