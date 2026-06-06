// DASPi-module-spherical-map.h
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "DASPi-config.h"
#include "DASPi-globallinearspace.h"
#include "DASPi-icosahedronspherespace.h"

namespace DASPi {

template<class SphereSpaceType, std::size_t ModuleIndex, unsigned int FacetIndex>
class ModuleSphericalMap {
public:
    static_assert(IcosahedronSphereSpace_t<SphereSpaceType>);

    static constexpr std::size_t expectedFacetIndex_ =
        SphereSpaceType::template ModuleFaceIndex<ModuleIndex>();

    static_assert(
        expectedFacetIndex_ == FacetIndex,
        "ModuleIndex does not map to this FacetIndex"
    );

    static constexpr std::size_t regionCount_ = NUM_REGIONS;

    /*
     * Current output space:
     *
     *   full sensor-linear image
     *
     * Later this can become the global spherical/panorama output space
     * without changing the UDP frame payload path.
     */
    static constexpr std::uint32_t outputWidth_ =
        static_cast<std::uint32_t>(sensorWidthValue_);

    static constexpr std::uint32_t outputHeight_ =
        static_cast<std::uint32_t>(sensorHeightValue_);

    static constexpr std::size_t outputSizeValue_ =
        static_cast<std::size_t>(sensorWidthValue_) *
        static_cast<std::size_t>(sensorHeightValue_);

    static_assert(
        outputSizeValue_ <= std::numeric_limits<std::uint32_t>::max(),
        "ModuleSphericalMap output size does not fit in uint32_t"
    );

    static constexpr std::uint32_t outputSize_ =
        static_cast<std::uint32_t>(outputSizeValue_);

    using RegionMap =
        std::vector<std::uint32_t>;

    using RegionMaps =
        std::array<RegionMap, regionCount_>;

    RegionMaps regionGlobalIndices_{};

    template<class OverlapTopologyType>
    explicit ModuleSphericalMap(const OverlapTopologyType& topology)
    {
        Build(topology);
    }

    constexpr std::uint32_t OutputWidth() const noexcept
    {
        return outputWidth_;
    }

    constexpr std::uint32_t OutputHeight() const noexcept
    {
        return outputHeight_;
    }

    constexpr std::uint32_t OutputSize() const noexcept
    {
        return outputSize_;
    }

    const RegionMap& Region(std::size_t regionIndex) const
    {
        return regionGlobalIndices_.at(regionIndex);
    }

private:
    template<class OverlapTopologyType>
    void Build(const OverlapTopologyType& topology)
    {
        const auto& nonOverlapTopology =
            static_cast<
                const typename OverlapTopologyType::NonOverlapFacetTopology_t&
            >(topology);

        AppendFromIndexLinear(
            0,
            *nonOverlapTopology.indexLinearMax_,
            nonOverlapTopology.size()
        );

        for (std::size_t regionIndex = 1;
             regionIndex < regionCount_;
             ++regionIndex) {

            const std::size_t overlapIndex =
                regionIndex - 1;

            AppendFromIndexLinear(
                regionIndex,
                *topology.indexLinearMaxs_[overlapIndex],
                topology.size(overlapIndex)
            );
        }
    }

    template<class IndexLinear>
    void AppendFromIndexLinear(
        std::size_t regionIndex,
        const IndexLinear& indexLinear,
        std::size_t validCount)
    {
        if (regionIndex >= regionCount_) {
            throw std::out_of_range(
                "ModuleSphericalMap: regionIndex out of range"
            );
        }

        if (validCount > indexLinear.size()) {
            throw std::runtime_error(
                "ModuleSphericalMap: validCount exceeds indexLinear size"
            );
        }

        auto& dst =
            regionGlobalIndices_[regionIndex];

        dst.clear();
        dst.reserve(validCount);

        for (std::size_t i = 0; i < validCount; ++i) {
            /*
             * Placeholder mapping:
             *
             *   packed masked index -> full sensor-linear index
             *
             * Later replace this with:
             *
             *   sensor index
             *     -> sensor x/y
             *     -> camera ray
             *     -> world/global ray
             *     -> spherical/panorama output index
             */
            dst.push_back(
                static_cast<std::uint32_t>(indexLinear[i].value())
            );
        }
    }
};

} // namespace DASPi
