#pragma once
#include <span>
#include <cstddef> // std::byte
#include "DASPi-frameheader.h"

namespace DASPi{
    struct FramePacket {
        static constexpr size_t MAX_ALLOWED_FRAME_SIZE_{41943040};//approx 4k rgb = 40mib
        
        FrameHeader header_;
        std::vector<uint16_t> payload_;
        
        [[nodiscard]]
        std::span<const std::byte> payload_as_bytes() const noexcept {
            return std::as_bytes(std::span(payload_));
        }
    };
};//ending namespace DASPi
