// DASPi-sphericalshapefunction.h
#pragma once

#include "DASPi-pointdata.h"
#include "DASPi-directiondata.h"
#include "DASPi-sphericalspace.h"

namespace DASPi{

template<SphericalSpace_t Space>
class  SphericalTopology
	: public OverlapTopology<MakeOverlapSpace<Space::n_, Space::center_, Space::orientation_, Space::nonOverlapScale_>>
{
public:
	using facet_t = OverlapTopology::facet_t;
	using nonOverlapFacet_t = OverlapTopology::nonOverlapFacet_t;
	
private:
};

}//DASPi
