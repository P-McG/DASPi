//DASPi-facetspace.h
#pragma once

#include <concepts>
#include <cstddef>

#include "DASPi-pointdata.h"
#include "DASPi-orientationdata.h"

namespace DASPi {

template<DirectionData facetDirection>
struct OverlapSpace
{
    static constexpr unsigned int facetIndex_ = facetIndex;
    static constexpr std::size_t n_ = n;
};

template<class Space>
concept FacetSpace_t =
    requires {
		{ Space::facetIndex_ } -> std::convertible_to<unsigned int>;
        { Space::n_ } -> std::convertible_to<std::size_t>;
    };

template<class Space>
using MakeFacetSpace =
    FacetSpace<
	    Space::facetIndex_,
        Space::n_
    >;

} // namespace DASPi
