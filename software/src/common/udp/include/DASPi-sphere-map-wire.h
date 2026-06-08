// DASPi-sphere-map-wire.h
#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace DASPi {

inline constexpr std::uint32_t kSphereMapWireMagic =
    0x31504D53u; // "SMP1"

/*
 * Version 2:
 *
 *   Each uint32 map entry is:
 *
 *     bits  0..29 = output index
 *     bits 30..31 = Bayer channel
 */
inline constexpr std::uint32_t kSphereMapWireVersion =
    2;

inline constexpr std::size_t kSphereMapWireHeaderU32Count =
    5;

inline constexpr std::size_t kSphereMapWireHeaderWords =
    kSphereMapWireHeaderU32Count * 2; // uint16_t words

enum class SphereMapBayerChannel : std::uint32_t {
    Blue    = 0,
    Green   = 1,
    Red     = 2,
    Unknown = 3
};

inline constexpr std::uint32_t kSphereMapOutputIndexBits =
    30;

inline constexpr std::uint32_t kSphereMapOutputIndexMask =
    (1u << kSphereMapOutputIndexBits) - 1u;

inline constexpr std::uint32_t kSphereMapBayerChannelShift =
    kSphereMapOutputIndexBits;

inline std::uint32_t PackSphereMapEntry(
    std::uint32_t outputIndex,
    SphereMapBayerChannel channel)
{
    if (outputIndex > kSphereMapOutputIndexMask) {
        throw std::runtime_error(
            "PackSphereMapEntry: outputIndex exceeds 30-bit payload"
        );
    }

    return outputIndex |
           (static_cast<std::uint32_t>(channel)
            << kSphereMapBayerChannelShift);
}

inline std::uint32_t UnpackSphereMapOutputIndex(
    std::uint32_t packedEntry) noexcept
{
    return packedEntry & kSphereMapOutputIndexMask;
}

inline SphereMapBayerChannel UnpackSphereMapBayerChannel(
    std::uint32_t packedEntry) noexcept
{
    return static_cast<SphereMapBayerChannel>(
        packedEntry >> kSphereMapBayerChannelShift
    );
}

} // namespace DASPi
