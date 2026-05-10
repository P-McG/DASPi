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

template<
    template<unsigned int> class FacetSpace,
    std::size_t FacetsN,
    std::size_t... FacetIndices
>
struct SphereSpaceImpl<
    FacetSpace,
    FacetsN,
    std::index_sequence<FacetIndices...>
>
{
    static_assert(
        FacetsN > 0,
        "SphereSpaceImpl requires at least one facet."
    );

    static_assert(
        sizeof...(FacetIndices) == FacetsN,
        "Facet index sequence size must match FacetsN."
    );

    static_assert(
        ((FacetIndices < FacetsN) && ...),
        "Facet index sequence contains an out-of-range facet index."
    );

    static constexpr std::size_t facetsN_{FacetsN};

    using facet_index_sequence_t =
        std::index_sequence<FacetIndices...>;

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

    static inline constexpr std::array<std::size_t, facetsN_>
    facetIndices_{{
        FacetIndices...
    }};

    template<std::size_t FacetIndex>
    static consteval bool IsValidFacetIndex()
    {
        return FacetIndex < facetsN_;
    }
};

using IcosahedronSphereSpace =
    SphereSpaceImpl<
        IcosahedronSpace,
        detail::IcosahedronTables::facetsN_,
        std::make_index_sequence<detail::IcosahedronTables::facetsN_>
    >;

template<class Space>
concept IcosahedronSphereSpace_t =
    requires {
        { Space::facetsN_ } -> std::convertible_to<std::size_t>;
        { Space::verticesPerFaceN_ } -> std::convertible_to<std::size_t>;
        { Space::dim_ } -> std::convertible_to<std::size_t>;

        typename Space::facet_index_sequence_t;

        typename Space::template FacetSpace_t<0>;
        typename Space::template SubSpace_t<0>;

        { Space::facetIndices_ } -> std::convertible_to<
            const std::array<std::size_t, Space::facetsN_>&
        >;
    };

} // namespace DASPi
