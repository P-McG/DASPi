// DASPi-shape-config.h
#pragma once

#include "DASPi-config.h"
#include "DASPi-overlapshapefunction.h"
#include "DASPi-shapefunctiondatapacket.h"


namespace DASPi {

inline constexpr PointData defaultShapeCenter{
    static_cast<size_t>(0.5 * sensorWidthValue_),
    static_cast<size_t>(0.5 * sensorHeightValue_)
};

//template <size_t n>
//inline constexpr DirectionData defaultShapeDirection{
    //1 * static_cast<long>((1.0 / 2.0) * sensorHeightValue_ *
                          //sin(2.0 * std::numbers::pi * orientationValue_ / n)),
    //1 * static_cast<long>((1.0 / 2.0) * sensorHeightValue_ *
                          //cos(2.0 * std::numbers::pi * orientationValue_ / n))
//};

//The distance from a vertex to the centroid for a triangle is: sqrt(3)/3*a
template <size_t n>
inline constexpr DirectionData defaultShapeDirection{
    1 * static_cast<long>((pow(3.0, 1.0 / 2.0) / 3.0) * sensorHeightValue_ *
                          sin(2.0 * std::numbers::pi * orientationValue_ / n)),
    1 * static_cast<long>((pow(3.0, 1.0 / 2.0) / 3.0) * sensorHeightValue_ *
                          cos(2.0 * std::numbers::pi * orientationValue_ / n))
};

template <size_t n>
using sf_t = OverlapShapeFunction<
    n,
    defaultShapeCenter,
    defaultShapeDirection<n>,
    overlapScaleValue_
>;

template <size_t n>
using sfdp_t = ShapeFunctionDataPacket<
    n,
    defaultShapeCenter,
    defaultShapeDirection<n>,
    overlapScaleValue_
>;

} // namespace DASPi
