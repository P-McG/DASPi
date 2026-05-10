// DASPi-regularpolygonalspace.h

#pragma once

#include <concepts>
#include <cstddef>

#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"

namespace DASPi {

template<std::size_t n, MaskOrientationData maskOrientation, SensorOrientationData sensorOrientation>
struct RegularPolygonalSpace
{
	using SubSpace_t = GlobalLinearSpace<sensorOrientation>;
	
    static constexpr std::size_t n_{n};
    static constexpr MaskOrientationData maskOrientation_{maskOrientation};
    static constexpr SensorOrientationData sensorOrientation_{sensorOrientation};
    static constexpr Matrix3dData ImageTransformMatrix = IdentityMatrix3d();
};

template<class Space>
concept RegularPolygonalSpace_t =
    requires {
        { Space::n_ } -> std::convertible_to<std::size_t>;
		{ Space::maskOrientation_ } -> std::same_as<const MaskOrientationData&>;
		{ Space::sensorOrientation_ } -> std::same_as<const SensorOrientationData&>;
    };

template<class Space>
using MakeRegularPolygonalSpace =
    RegularPolygonalSpace<
        Space::n_,
        Space::maskOrientation_,
        Space::sensorOrientation_
    >;

} // namespace DASPi
