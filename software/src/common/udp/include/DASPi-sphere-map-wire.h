// DASPi-sphere-map-wire.h
#pragma once

#include <cstddef>
#include <cstdint>

namespace DASPi {

inline constexpr std::uint32_t kSphereMapWireMagic =
    0x31504D53u; // "SMP1"

inline constexpr std::uint32_t kSphereMapWireVersion =
    1;

inline constexpr std::size_t kSphereMapWireHeaderU32Count =
    5;

inline constexpr std::size_t kSphereMapWireHeaderWords =
    kSphereMapWireHeaderU32Count * 2; // uint16_t words

} // namespace DASPi
