// DASPi-module-spherical-map.h
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace DASPi {

struct GlobalSphereSample {
    std::uint32_t panoIndex{};
    std::uint16_t panoX{};
    std::uint16_t panoY{};
    std::uint16_t sensorIndex{};
};

template<
    class SphereSpaceType,
    std::size_t ModuleIndex,
    unsigned int FacetIndex
>
class ModuleSphericalMap {
public:
    static_assert(IcosahedronSphereSpace_t<SphereSpaceType>);

    static constexpr std::size_t expectedFacetIndex =
        SphereSpaceType::template ModuleFaceIndex<ModuleIndex>();

    static_assert(
        expectedFacetIndex == FacetIndex,
        "ModuleIndex does not map to this FacetIndex"
    );

    using FacetSpaceType =
        typename SphereSpaceType::template FacetSpace_t<FacetIndex>;

    using OverlapSpaceType =
        typename FacetSpaceType::SubSpace_t;

    using OverlapTopologyType =
        OverlapTopology<OverlapSpaceType>;

    static constexpr std::size_t regionCount =
        FacetSpaceType::verticesPerFaceN_ + 1;

    std::array<std::vector<GlobalSphereSample>, regionCount> regionMaps;

    explicit ModuleSphericalMap(const OverlapTopologyType& topology)
    {
        build(topology);
    }

private:
    void build(const OverlapTopologyType& topology)
    {
        /*
         * Build:
         *
         *   packed region index
         *       -> original sensor linear index
         *       -> sensor x/y
         *       -> camera ray
         *       -> global/world ray
         *       -> spherical/equirect pano x/y/index
         *
         * This table is constant for this ModuleIndex + FacetIndex.
         */
    }
};

} // namespace DASPi
