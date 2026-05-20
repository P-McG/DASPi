#pragma once

#include <concepts>
#include <cstddef>
#include <numbers>

#include "DASPi-pointdata2di.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-rotation.h"

namespace DASPi {

static constexpr std::size_t sensorWidthValue_  = 1456;
static constexpr std::size_t sensorHeightValue_ = 1088;

template<SensorOrientationData SensorOrientation>
struct GlobalLinearSpace
{
    static constexpr PointData2dI sensorCenter_{
        sensorWidthValue_ / 2,
        sensorHeightValue_ / 2
    };

    static constexpr SensorOrientationData sensorOrientation_{
        SensorOrientation
    };

    static constexpr Matrix3dData ImageTransformMatrix{
        RotationZAroundPointFromSinCos(
            sensorCenter_.x_,
            sensorCenter_.y_,
            sensorOrientation_.cosValue_,
            sensorOrientation_.sinValue_
        )
    };
};

template<class Space>
concept GlobalLinearSpace_t =
    requires {
        { Space::sensorCenter_ } ->
            std::same_as<const PointData2dI&>;

        { Space::sensorOrientation_ } ->
            std::same_as<const SensorOrientationData&>;

        { Space::ImageTransformMatrix } ->
            std::same_as<const Matrix3dData&>;
    };

template<class Space>
using MakeGlobalLinearSpace =
    GlobalLinearSpace<
        Space::sensorOrientation_
    >;

} // namespace DASPi
