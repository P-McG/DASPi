// DASPi-topology_edge.h
#pragma once
#include <array>

namespace DASPi{
struct TopologyEdge {
    std::array<std::size_t, 2> vertices{}; // undirected edge, sorted
    std::array<int, 2> incidentFaces{ -1, -1 };
};
};//DASPi
