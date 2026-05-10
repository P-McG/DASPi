// DASPi-shape-config.h
#pragma once

#include "DASPi-config.h"
#include "DASPi-overlaptopology.h"
#include "DASPi-icosahedronspace.h"

#include "DASPi-icosahedron_topology.h"
#include "DASPi-icosahedronspherespace.h"
#include "DASPi-icosahedronspheretopology.h"
#include "DASPi-topologydatapacket.h"


namespace DASPi {

//inline constexpr PointData2dI defaultShapeCenter{
    //static_cast<size_t>(0.5 * sensorWidthValue_),
    //static_cast<size_t>(0.5 * sensorHeightValue_)
//};

//template <size_t n>
//inline constexpr OrientationData defaultShapeOrientation{
    //1 * static_cast<long>((1.0 / 2.0) * sensorHeightValue_ *
                          //sin(2.0 * std::numbers::pi * orientationValue_ / n)),
    //1 * static_cast<long>((1.0 / 2.0) * sensorHeightValue_ *
                          //cos(2.0 * std::numbers::pi * orientationValue_ / n))
//};

//The distance from a vertex to the centroid for a triangle is: sqrt(3)/3*a
//template <size_t n>
//inline constexpr OrientationData defaultShapeOrientation{
    //1 * static_cast<long>((pow(3.0, 1.0 / 2.0) / 3.0) * sensorHeightValue_ *
                          //sin(2.0 * std::numbers::pi * orientationValue_ / n)),
    //1 * static_cast<long>((pow(3.0, 1.0 / 2.0) / 3.0) * sensorHeightValue_ *
                          //cos(2.0 * std::numbers::pi * orientationValue_ / n))
//};

//The following used the full Topology to be instanated
using tpgy_t = IcosahedronSphereTopology<IcosahedronSphereSpace>;

//The following uses the Topology that will be transmitted to the computemodule
template <unsigned int FacetIndex>
using tpgydp_t = TopologyDataPacket<IcosahedronSpace<FacetIndex>>;

} // namespace DASPi
