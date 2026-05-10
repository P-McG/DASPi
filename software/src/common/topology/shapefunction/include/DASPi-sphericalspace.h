//// DASPi-sphericalspace.h
//#pragma once
//#include "DASPi-pointdata.h"
//#include "DASPi-orientationdata.h"

//namespace DASPi{
	
//template<class Space>
//using SphericalSpace_t = SphericalSpace<Space::n_, Space::center_, Space::orientation_, Space::nonOverlapScale_>; 
	
//template<size_t n, PointData2dI center, OrientationData orientation, double nonOverlapScale>
//struct SphericalSpace
//{
	//static constexpr std::size_t n_ = n;
	//static constexpr PointData2dI center_ = center;
	//static constexpr OrientationData orientation_ = orientation; 
	//static constexpr double nonOverlapScale_ = nonOverlapScale;
//};
//}//DASPi

#pragma once

#include <concepts>
#include <cstddef>

//#include "DASPi-pointdata.h"
//#include "DASPi-orientationdata.h"
//#include "DASPi-globalaxes.h"

namespace DASPi {

template<unsigned int FacetIndex>
struct SphericalSpace
{
	static constexpr unsigned int facetIndex_ = FacetIndex;
};

template<class Space>
concept SphericalSpace_t =
    requires {
		{ Space::facetIndex_ } -> std::convertible_to<unsigned int>;
    };

template<class Space>
using MakeSphericalSpace =
    SphericalSpace<
	    Space::facetIndex_
    >;

} // namespace DASPi
