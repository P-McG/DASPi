// DASPi-module-spherical-map.h
#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <cstddef>
#include <stdexcept>

#include "DASPi-config.h"
#include "DASPi-icosahedronspherespace.h"

namespace DASPi {

template<class SphereSpaceType, std::size_t ModuleIndex, unsigned int FacetIndex>
class ModuleSphericalMap {
public:
    static_assert(IcosahedronSphereSpace_t<SphereSpaceType>);

    static constexpr std::size_t expectedFacetIndex =
        SphereSpaceType::template ModuleFaceIndex<ModuleIndex>();

    static_assert(
        expectedFacetIndex == FacetIndex,
        "ModuleIndex does not map to this FacetIndex"
    );

    static constexpr std::size_t regionCount_ = NUM_REGIONS;

    std::array<std::vector<std::uint32_t>, regionCount_> regionGlobalIndices_{};

    template<class OverlapTopologyType>
    explicit ModuleSphericalMap(const OverlapTopologyType& topology)
    {
        Build(topology);
    }

    const std::vector<std::uint32_t>& Region(std::size_t regionIndex) const
    {
        return regionGlobalIndices_.at(regionIndex);
    }

private:
    template<class OverlapTopologyType>
    void Build(const OverlapTopologyType& topology)
    {
        const auto& nonOverlapTopology =
            static_cast<const typename OverlapTopologyType::NonOverlapFacetTopology_t&>(
                topology
            );
    
        AppendFromIndexLinear(
            0,
            *nonOverlapTopology.indexLinearMax_,
            nonOverlapTopology.size()
        );
    
        for (std::size_t r = 1; r < regionCount_; ++r) {
            const std::size_t overlapIndex = r - 1;
    
            AppendFromIndexLinear(
                r,
                *topology.indexLinearMaxs_[overlapIndex],
                topology.size(overlapIndex)
            );
        }
    }
    
    template<class IndexLinear>
    void AppendFromIndexLinear(
        std::size_t regionIndex,
        const IndexLinear& indexLinear,
        std::size_t validCount
    )
    {
        auto& dst = regionGlobalIndices_[regionIndex];
        dst.clear();
        dst.reserve(validCount);
    
        if (validCount > indexLinear.size()) {
            throw std::runtime_error("ModuleSphericalMap: validCount exceeds indexLinear size");
        }
    
        for (std::size_t i = 0; i < validCount; ++i) {
            dst.push_back(
                static_cast<std::uint32_t>(indexLinear[i].value())
            );
        }
    }
};

} // namespace DASPi
