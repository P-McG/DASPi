#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "DASPi-config.h"
#include "DASPi-messages.h"

namespace DASPi {

constexpr uint32_t MAGIC_NUMBER = 0xCAFEBABE;

#pragma pack(push, 1)
struct FrameHeader {
    uint32_t magic_;           // Magic number
    GainMsg gainMsg_;           // Gain + frame metadata

    uint32_t payloadSize_;     // Total payload bytes

    std::array<uint32_t, NUM_REGIONS> regionSizes_;  // per-region element counts

    uint32_t checksum_;        // checksum over payload
};
#pragma pack(pop)

// Safety checks
static_assert(std::is_standard_layout_v<FrameHeader>, "FrameHeader must be standard layout");
static_assert(std::is_trivially_copyable_v<FrameHeader>, "FrameHeader must be trivially copyable");

static_assert(std::is_standard_layout_v<GainMsg>, "GainMsg must be standard layout");
static_assert(std::is_trivially_copyable_v<GainMsg>, "GainMsg must be trivially copyable");

static_assert(sizeof(uint32_t) == 4);

} // namespace DASPi
