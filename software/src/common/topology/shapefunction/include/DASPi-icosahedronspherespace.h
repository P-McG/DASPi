#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <utility>

#include "DASPi-icosahedronspace.h"

namespace DASPi {

template<
    template<unsigned int> class FacetSpace,
    std::size_t FacetsN,
    class IndexSequence
>
struct SphereSpaceImpl;

// SphereSpaceImpl
/*
 * the full sphere remains simple:

using IcosahedronSphereSpace =
    SphereSpaceImpl<
        IcosahedronSpace,
        detail::IcosahedronTables::facetsN_,
        std::make_index_sequence<detail::IcosahedronTables::facetsN_>
    >;

And a compute-module-owned subset can be expressed like this:

using IcosahedronModule0SphereSpace =
    SphereSpaceImpl<
        IcosahedronSpace,
        detail::IcosahedronTables::facetsN_,
        std::index_sequence<0, 1, 2, 3, 4>
    >;
 */
template<
    template<unsigned int> class FacetSpace,
    std::size_t TotalFacetsN,
    std::size_t... ModuleFaceIndices
>
struct SphereSpaceImpl<
    FacetSpace,
    TotalFacetsN,
    std::index_sequence<ModuleFaceIndices...>
>
{
    static_assert(
        TotalFacetsN > 0,
        "SphereSpaceImpl requires at least one total facet."
    );

    static_assert(
        sizeof...(ModuleFaceIndices) > 0,
        "SphereSpaceImpl requires at least one module face."
    );

    static_assert(
        ((ModuleFaceIndices < TotalFacetsN) && ...),
        "Module face index sequence contains an out-of-range facet index."
    );

    static constexpr std::size_t totalFacetsN_{TotalFacetsN};
    static constexpr std::size_t moduleFacesN_{sizeof...(ModuleFaceIndices)};

    using ModuleFaceIndices_t =
        std::index_sequence<ModuleFaceIndices...>;
    
    using module_face_index_sequence_t =
        std::index_sequence<ModuleFaceIndices...>;

    template<std::size_t FacetIndex>
    using FacetSpace_t =
        FacetSpace<static_cast<unsigned int>(FacetIndex)>;

    template<std::size_t FacetIndex>
    using SubSpace_t =
        typename FacetSpace_t<FacetIndex>::SubSpace_t;

    static constexpr std::size_t verticesPerFaceN_{
        FacetSpace_t<0>::verticesPerFaceN_
    };

    static constexpr std::size_t dim_{
        FacetSpace_t<0>::dim_
    };

    static inline constexpr std::array<std::size_t, moduleFacesN_>
    moduleFaceIndices_{{
        ModuleFaceIndices...
    }};

    template<std::size_t FacetIndex>
    static consteval bool IsValidFacetIndex()
    {
        return FacetIndex < totalFacetsN_;
    }

    template<std::size_t FacetIndex>
    static consteval bool ContainsModuleFaceIndex()
    {
        return ((FacetIndex == ModuleFaceIndices) || ...);
    }
};

template<std::size_t... ModuleFaceIndices>
using IcosahedronModuleSphereSpace =
    SphereSpaceImpl<
        IcosahedronSpace,
        detail::IcosahedronTables::facetsN_,
        std::index_sequence<ModuleFaceIndices...>
    >;

using IcosahedronSphereSpace =
    SphereSpaceImpl<
        IcosahedronSpace,
        detail::IcosahedronTables::facetsN_,
        std::make_index_sequence<detail::IcosahedronTables::facetsN_>
    >;

template<class Space>
concept IcosahedronSphereSpace_t =
    requires {
        { Space::totalFacetsN_ } -> std::convertible_to<std::size_t>;
        { Space::moduleFacesN_ } -> std::convertible_to<std::size_t>;
        { Space::verticesPerFaceN_ } -> std::convertible_to<std::size_t>;
        { Space::dim_ } -> std::convertible_to<std::size_t>;

        typename Space::module_face_index_sequence_t;

        typename Space::template FacetSpace_t<0>;
        typename Space::template SubSpace_t<0>;

        { Space::moduleFaceIndices_ } -> std::convertible_to<
            const std::array<std::size_t, Space::moduleFacesN_>&
        >;
    }
    && (Space::totalFacetsN_ > 0)
    && (Space::moduleFacesN_ > 0)
    && (Space::moduleFacesN_ <= Space::totalFacetsN_);

} // namespace DASPi
