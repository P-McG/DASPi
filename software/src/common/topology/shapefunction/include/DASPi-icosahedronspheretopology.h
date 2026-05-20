// DASPi-icosahedronspheretopology.h
#pragma once

#include <concepts>
#include <cstddef>

#include "DASPi-icosahedronspherespace.h"
#include "DASPi-icosahedrontopology.h"
#include "DASPi-overlaptopology.h"

namespace DASPi {

template<class Space, class ModuleFaceIndexSequence>
class IcosahedronSphereTopologyImpl;

template<class Space, std::size_t... ModuleFaceIndices>
class IcosahedronSphereTopologyImpl<
    Space,
    std::index_sequence<ModuleFaceIndices...>
> : public IcosahedronTopology<
        typename Space::template FacetSpace_t<ModuleFaceIndices>
    >...
{
public:
    using Space_t = Space;

    static constexpr std::size_t totalFacetsN_ =
        Space::totalFacetsN_;

    static constexpr std::size_t moduleFacesN_ =
        Space::moduleFacesN_;

    static constexpr std::size_t verticesPerFaceN_ =
        Space::verticesPerFaceN_;

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

    template<std::size_t FacetIndex>
    static consteval bool ContainsModuleFaceIndex()
    {
        return Space::template ContainsModuleFaceIndex<FacetIndex>();
    }
};

template<class Space>
requires IcosahedronSphereSpace_t<Space>
class IcosahedronSphereTopology
    : public IcosahedronSphereTopologyImpl<
          Space,
          typename Space::module_face_index_sequence_t
      >
{
public:
    using Base_t =
        IcosahedronSphereTopologyImpl<
            Space,
            typename Space::module_face_index_sequence_t
        >;

    using Base_t::Base_t;
};

} // namespace DASPi
