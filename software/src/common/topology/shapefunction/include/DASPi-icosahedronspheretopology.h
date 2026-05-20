// DASPi-icosahedronspheretopology.h
#pragma once

#include <concepts>
#include <cstddef>

#include "DASPi-icosahedronspherespace.h"
#include "DASPi-icosahedrontopology.h"
#include "DASPi-overlaptopology.h"

namespace DASPi {

template<class Space>
requires IcosahedronSphereSpace_t<Space>
class IcosahedronSphereTopology
{
public:
    using Space_t = Space;

    static constexpr std::size_t facetsN_{
        Space::facetsN_
    };

    static constexpr std::size_t verticesPerFaceN_{
        Space::verticesPerFaceN_
    };

    template<std::size_t FacetIndex>
    using FacetSpace_t =
        typename Space::template FacetSpace_t<FacetIndex>;

    template<std::size_t FacetIndex>
    using OverlapSpace_t =
        typename Space::template SubSpace_t<FacetIndex>;

    template<std::size_t FacetIndex>
    using FacetTopology_t =
        IcosahedronTopology<FacetSpace_t<FacetIndex>>;

    template<std::size_t FacetIndex>
    using OverlapTopology_t =
        OverlapTopology<OverlapSpace_t<FacetIndex>>;
};

} // namespace DASPi
