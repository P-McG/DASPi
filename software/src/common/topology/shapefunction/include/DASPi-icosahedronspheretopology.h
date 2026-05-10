// DASPi-icosahedronspheretopology.h
#pragma once
#include "DASPi-icosahedronspherespace.h"


namespace DASPi{


template<class Space>
requires IcosahedronSphereSpace_t<Space>
class IcosahedronSphereTopology
{
public:
	using Space_t = Space;
    static constexpr std::size_t facetsN_{Space::facetsN_};

    // topology implementation...
};
    
}//DASPi
