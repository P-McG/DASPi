// DASPi-topology_edge.h
#pragma once

#include <array>
#include <cstddef>

namespace DASPi {

struct TopologyEdge {
    // Undirected edge, sorted as {minVertex, maxVertex}.
    std::array<std::size_t, 2> vertices{};

    // The two faces touching this edge.
    // For a closed icosahedron, both entries should be valid.
    std::array<int, 2> incidentFaces{ -1, -1 };

    // The local edge index inside each incident face.
    // For a triangle face, local edge e means:
    // face[e] -> face[(e + 1) % 3]
    std::array<int, 2> incidentLocalEdges{ -1, -1 };
};

} // namespace DASPi
