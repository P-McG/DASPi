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

using ConfiguredSphereSpace =
    IcosahedronModuleSphereSpace<9, 19, 2, 18>;

using tpgy_t =
    IcosahedronSphereTopology<ConfiguredSphereSpace>;

template <unsigned int FacetIndex>
using tpgydp_t =
    TopologyDataPacket<
        typename tpgy_t::template OverlapSpace_t<FacetIndex>
    >;

} // namespace DASPi
