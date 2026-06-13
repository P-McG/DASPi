#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "DASPi-config.h"
#include "DASPi-message-header.h"
#include "DASPi-gain-msg.h"
#include "DASPi-gain-reply.h"

namespace DASPi {
		
		
enum class WirePixelFormat : uint16_t {
    Uint16LE          = 1,
    BayerRaw10Packed = 2,
};

inline constexpr uint16_t ToWirePixelFormatValue(WirePixelFormat f)
{
    return static_cast<uint16_t>(f);
}

inline constexpr WirePixelFormat ToWirePixelFormat(uint16_t v)
{
    return static_cast<WirePixelFormat>(v);
}


constexpr uint32_t MAGIC_NUMBER = 0xCAFEBABE;

#pragma pack(push, 1)
struct FrameHeader {
    uint32_t magic_;

    uint16_t headerVersion_{2};
    uint16_t wirePixelFormat_{
        ToWirePixelFormatValue(WirePixelFormat::Uint16LE)
    };

    GainMsg gainMsg_;

    uint32_t payloadSize_;  // total wire payload bytes

    /*
     * Pixel counts after decode.
     *
     * For Uint16LE:
     *   regionPayloadBytes_[i] == regionSizes_[i] * sizeof(uint16_t)
     *
     * For BayerRaw10Packed:
     *   regionPayloadBytes_[i] == packed RAW10 bytes for regionSizes_[i] pixels
     */
    std::array<uint32_t, NUM_REGIONS> regionSizes_;
    std::array<uint32_t, NUM_REGIONS> regionPayloadBytes_;

    uint32_t checksum_;     // checksum over wire payload bytes
};
#pragma pack(pop)

// Safety checks
static_assert(std::is_standard_layout_v<FrameHeader>, "FrameHeader must be standard layout");
static_assert(std::is_trivially_copyable_v<FrameHeader>, "FrameHeader must be trivially copyable");

static_assert(std::is_standard_layout_v<GainMsg>, "GainMsg must be standard layout");
static_assert(std::is_trivially_copyable_v<GainMsg>, "GainMsg must be trivially copyable");

static_assert(sizeof(uint32_t) == 4);

} // namespace DASPi
