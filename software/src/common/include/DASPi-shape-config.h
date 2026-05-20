// DASPi-shape-config.h
#pragma once

#include "DASPi-config.h"
#include "DASPi-overlaptopology.h"
#include "DASPi-icosahedronspace.h"

#include "DASPi-icosahedrontopology.h"
#include "DASPi-icosahedronspherespace.h"
#include "DASPi-icosahedronspheretopology.h"
#include "DASPi-topologydatapacket.h"


namespace DASPi {

//The following used the full Topology to be instantiated
//using tpgy_t = IcosahedronSphereTopology<IcosahedronSphereSpace>;
using tpgy_t =  IcosahedronSphereTopology<IcosahedronModuleSphereSpace<0, 1>>;

//The following uses the Topology that will be transmitted to the computemodule
//template <unsigned int FacetIndex>
//using tpgydp_t = TopologyDataPacket<typename IcosahedronSphereSpace::SubSpace_t<FacetIndex>>;
template <unsigned int FacetIndex>
using tpgydp_t =
    TopologyDataPacket<typename tpgy_t::template OverlapSpace_t<FacetIndex>>;

} // namespace DASPi
