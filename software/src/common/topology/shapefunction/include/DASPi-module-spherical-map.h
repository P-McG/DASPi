// DASPi-module-spherical-map.h
#pragma once

#include <array>
#include <cstdint>
#include <vector>

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

        AppendFromIndexLinear(0, *nonOverlapTopology.indexLinearMax_);

        for (std::size_t r = 1; r < regionCount_; ++r) {
            AppendFromIndexLinear(r, *topology.indexLinearMaxs_[r - 1]);
        }
    }

    template<class IndexLinear>
    void AppendFromIndexLinear(std::size_t regionIndex, const IndexLinear& indexLinear)
    {
        auto& dst = regionGlobalIndices_[regionIndex];
        dst.clear();
        dst.reserve(indexLinear.size());

        for (const auto& sensorIndex : indexLinear) {
            /*
             * Temporary global index:
             *   packed local masked pixel -> original full sensor linear index.
             *
             * Later replace this with:
             *   sensor index -> ray -> global spherical/pano index.
             */
            dst.push_back(static_cast<std::uint32_t>(sensorIndex.value()));
        }
    }
};

} // namespace DASPi
