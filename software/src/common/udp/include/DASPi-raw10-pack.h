//DASPi-raw10-pack.h

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace DASPi {

inline constexpr std::size_t Raw10PackedByteCount(std::size_t pixelCount)
{
    return ((pixelCount + 3u) / 4u) * 5u;
}

inline std::uint16_t Quantize16ToRaw10(std::uint16_t v)
{
    return static_cast<std::uint16_t>(
        std::min<std::uint32_t>(
            (static_cast<std::uint32_t>(v) + 32u) >> 6u,
            1023u
        )
    );
}

inline std::uint16_t ExpandRaw10To16(std::uint16_t v10)
{
    v10 &= 0x03FFu;

    /*
     * Bit replication gives 0..65535-like scaling:
     *
     *   10-bit 1023 -> 65535
     */
    return static_cast<std::uint16_t>((v10 << 6u) | (v10 >> 4u));
}

inline void AppendRaw10Packed(std::span<const std::uint16_t> src,
                              std::vector<std::uint8_t>& out)
{
    out.reserve(out.size() + Raw10PackedByteCount(src.size()));

    std::size_t i = 0;

    while (i < src.size()) {
        const std::uint16_t p0 =
            i < src.size() ? Quantize16ToRaw10(src[i++]) : 0;

        const std::uint16_t p1 =
            i < src.size() ? Quantize16ToRaw10(src[i++]) : 0;

        const std::uint16_t p2 =
            i < src.size() ? Quantize16ToRaw10(src[i++]) : 0;

        const std::uint16_t p3 =
            i < src.size() ? Quantize16ToRaw10(src[i++]) : 0;

        /*
         * MIPI-style RAW10 layout:
         *
         *   byte0 = p0[9:2]
         *   byte1 = p1[9:2]
         *   byte2 = p2[9:2]
         *   byte3 = p3[9:2]
         *   byte4 = p0[1:0] | p1[1:0]<<2 | p2[1:0]<<4 | p3[1:0]<<6
         */
        out.push_back(static_cast<std::uint8_t>((p0 >> 2u) & 0xFFu));
        out.push_back(static_cast<std::uint8_t>((p1 >> 2u) & 0xFFu));
        out.push_back(static_cast<std::uint8_t>((p2 >> 2u) & 0xFFu));
        out.push_back(static_cast<std::uint8_t>((p3 >> 2u) & 0xFFu));

        out.push_back(
            static_cast<std::uint8_t>(
                ((p0 & 0x03u) << 0u) |
                ((p1 & 0x03u) << 2u) |
                ((p2 & 0x03u) << 4u) |
                ((p3 & 0x03u) << 6u)
            )
        );
    }
}

inline bool AppendRaw10UnpackedExpanded16(std::span<const std::uint8_t> src,
                                          std::size_t pixelCount,
                                          std::vector<std::uint16_t>& out)
{
    if (src.size() != Raw10PackedByteCount(pixelCount)) {
        return false;
    }

    out.reserve(out.size() + pixelCount);

    std::size_t si = 0;
    std::size_t produced = 0;

    while (produced < pixelCount) {
        const std::uint8_t b0 = src[si++];
        const std::uint8_t b1 = src[si++];
        const std::uint8_t b2 = src[si++];
        const std::uint8_t b3 = src[si++];
        const std::uint8_t b4 = src[si++];

        const std::uint16_t p0 =
            static_cast<std::uint16_t>((static_cast<std::uint16_t>(b0) << 2u) |
                                       ((b4 >> 0u) & 0x03u));

        const std::uint16_t p1 =
            static_cast<std::uint16_t>((static_cast<std::uint16_t>(b1) << 2u) |
                                       ((b4 >> 2u) & 0x03u));

        const std::uint16_t p2 =
            static_cast<std::uint16_t>((static_cast<std::uint16_t>(b2) << 2u) |
                                       ((b4 >> 4u) & 0x03u));

        const std::uint16_t p3 =
            static_cast<std::uint16_t>((static_cast<std::uint16_t>(b3) << 2u) |
                                       ((b4 >> 6u) & 0x03u));

        if (produced++ < pixelCount) out.push_back(ExpandRaw10To16(p0));
        if (produced++ < pixelCount) out.push_back(ExpandRaw10To16(p1));
        if (produced++ < pixelCount) out.push_back(ExpandRaw10To16(p2));
        if (produced++ < pixelCount) out.push_back(ExpandRaw10To16(p3));
    }

    return true;
}

} // namespace DASPi
