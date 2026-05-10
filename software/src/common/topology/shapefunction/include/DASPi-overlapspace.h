//// DASPi-overlapspace.h
//#pragma once
//#include "DASPi-pointdata.h"
//#include "DASPi-orientationdata.h"

//namespace DASPi{
	
//template<class Space>
//using OverlapSpace_t = OverlapSpace_t<Space::n_, Space::center_, Space::orientation_, Space::nonOverlapScale_>; 
	
//template<size_t n, PointData2dI center, OrientationData orientation, double nonOverlapScale>
//struct OverlapSpace
//{
	//static constexpr std::size_t n_ = n;
	//static constexpr PointData2dI center_ = center;
	//static constexpr OrientationData orientation_ = orientation; 
	//static constexpr double nonOverlapScale_ = nonOverlapScale;
//};
//}//DASPi

/*
  OverlapSpace<values...>  -> stores compile-time space data
  OverlapSpace_t<Space>    -> concept checking that a type behaves like overlap space
  MakeOverlapSpace<Space>  -> converts a compatible space-like type into canonical OverlapSpace
 */
#pragma once

#include <concepts>
#include <cstddef>

#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"

namespace DASPi {

template<std::size_t n, MaskOrientationData maskOrientation, SensorOrientationData sensorOrientation>
struct OverlapSpace
{
	using SubSpace_t = RegularPolygonalSpace<n, maskOrientation, sensorOrientation>;
	
    static constexpr std::size_t n_{n};
    static constexpr MaskOrientationData maskOrientation_{maskOrientation};
    static constexpr SensorOrientationData sensorOrientation_{sensorOrientation};
    static constexpr double nonOverlapScale_ {0.75};
    static constexpr Matrix3dData ImageTransformMatrix = IdentityMatrix3d();
};

template<class Space>
concept OverlapSpace_t =
    requires {
        { Space::n_ } -> std::convertible_to<std::size_t>;
		{ Space::maskOrientation_ } -> std::same_as<const MaskOrientationData&>;
		{ Space::sensorOrientation_ } -> std::same_as<const SensorOrientationData&>;
    };

template<class Space>
using MakeOverlapSpace =
    OverlapSpace<
        Space::n_,
        Space::maskOrientations_,
        Space::sensorOrientations_
    >;

} // namespace DASPi
