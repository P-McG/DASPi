// DASPi-networkaddressplan.h
#pragma once
#include <vector>

namespace DASPi{
struct NetworkAddressPlan {
    in_addr_t client{};
    std::vector<in_addr_t> servers;
};
}//DASPi
