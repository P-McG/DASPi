// DASPi-mesh_topology.h
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include "DASPi-topology_edge.h"
#include "DASPi-face_adjacency_entry.h"

namespace DASPi {

template <std::size_t N>
struct MeshTopology {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<std::size_t, N>> faces;

    // Derived connectivity
    std::vector<TopologyEdge> edges;
    std::vector<std::array<int, N>> faceEdgeIndices;
    std::vector<std::array<int, N>> faceNeighborIndices;
    std::vector<std::array<FaceAdjacencyEntry, N>> faceAdjacency;
};

template <std::size_t N>
inline std::array<std::size_t, 2> MakeUndirectedEdge(std::size_t a, std::size_t b)
{
    if (a < b) {
        return {a, b};
    }
    return {b, a};
}

template <std::size_t N>
inline void BuildTopologyAdjacency(MeshTopology<N>& topo)
{
    topo.edges.clear();
    topo.faceEdgeIndices.clear();
    topo.faceNeighborIndices.clear();
    topo.faceAdjacency.clear();

    topo.faceEdgeIndices.resize(topo.faces.size());
    topo.faceNeighborIndices.resize(topo.faces.size());
    topo.faceAdjacency.resize(topo.faces.size());

    for (std::size_t f = 0; f < topo.faces.size(); ++f) {
        topo.faceEdgeIndices[f].fill(-1);
        topo.faceNeighborIndices[f].fill(-1);
        for (std::size_t e = 0; e < N; ++e) {
            topo.faceAdjacency[f][e] = FaceAdjacencyEntry{};
            topo.faceAdjacency[f][e].localEdgeIndex = static_cast<int>(e);
        }
    }

    std::map<std::pair<std::size_t, std::size_t>, int> edgeMap;

    for (std::size_t faceIndex = 0; faceIndex < topo.faces.size(); ++faceIndex) {
        const auto& face = topo.faces[faceIndex];

        for (std::size_t localEdge = 0; localEdge < N; ++localEdge) {
            const std::size_t v0 = face[localEdge];
            const std::size_t v1 = face[(localEdge + 1) % N];

            const auto edge = MakeUndirectedEdge<N>(v0, v1);
            const auto key = std::make_pair(edge[0], edge[1]);

            auto it = edgeMap.find(key);
            if (it == edgeMap.end()) {
                TopologyEdge topoEdge;
                topoEdge.vertices = edge;
                topoEdge.incidentFaces[0] = static_cast<int>(faceIndex);

                const int edgeIndex = static_cast<int>(topo.edges.size());
                topo.edges.push_back(topoEdge);
                edgeMap.emplace(key, edgeIndex);
                topo.faceEdgeIndices[faceIndex][localEdge] = edgeIndex;
            } else {
                const int edgeIndex = it->second;
                TopologyEdge& topoEdge = topo.edges[static_cast<std::size_t>(edgeIndex)];

                if (topoEdge.incidentFaces[1] != -1) {
                    throw std::runtime_error("Non-manifold edge detected");
                }

                topoEdge.incidentFaces[1] = static_cast<int>(faceIndex);
                topo.faceEdgeIndices[faceIndex][localEdge] = edgeIndex;
            }
        }
    }

    // Fill neighbors and adjacency records
    for (std::size_t faceIndex = 0; faceIndex < topo.faces.size(); ++faceIndex) {
        for (std::size_t localEdge = 0; localEdge < N; ++localEdge) {
            const int edgeIndex = topo.faceEdgeIndices[faceIndex][localEdge];
            if (edgeIndex < 0) {
                throw std::runtime_error("Missing edge index while building adjacency");
            }

            const TopologyEdge& edge = topo.edges[static_cast<std::size_t>(edgeIndex)];
            const int f0 = edge.incidentFaces[0];
            const int f1 = edge.incidentFaces[1];

            int neighbor = -1;
            if (f0 == static_cast<int>(faceIndex)) {
                neighbor = f1;
            } else if (f1 == static_cast<int>(faceIndex)) {
                neighbor = f0;
            } else {
                throw std::runtime_error("Edge incident face mismatch");
            }

            topo.faceNeighborIndices[faceIndex][localEdge] = neighbor;
            topo.faceAdjacency[faceIndex][localEdge].neighborFaceIndex = neighbor;
            topo.faceAdjacency[faceIndex][localEdge].edgeIndex = edgeIndex;
        }
    }
}

} // namespace DASPi
