// DASPi-isocahedron_topology.h
#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include "DASPi-mesh_topology.h"

namespace DASPi {

class IcosahedronTopology {
public:
    MeshTopology<3> Make() const
    {
        MeshTopology<3> topo;

		// phi approx. 1.618033988749
        const double phi = (1.0 + std::sqrt(5.0)) * 0.5;

        topo.vertices = {
            {-1,  phi, 0}, { 1,  phi, 0}, {-1, -phi, 0}, { 1, -phi, 0},
            {0, -1,  phi}, {0,  1,  phi}, {0, -1, -phi}, {0,  1, -phi},
            { phi, 0, -1}, { phi, 0,  1}, {-phi, 0, -1}, {-phi, 0,  1},
        };

        const Eigen::Matrix3d R =
            Eigen::AngleAxisd(M_PI / 5.0, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(M_PI / 7.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

        for (auto& p : topo.vertices) {
            p = R * p;
        }

        topo.faces = {
            {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
            {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
            {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
            {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
        };

        BuildTopologyAdjacency(topo);
		
		if (topo.edges.size() != 30) {
			throw std::runtime_error("IcosahedronTopology expected 30 edges");
		}
		
		for (std::size_t faceIndex = 0; faceIndex < topo.faceNeighborIndices.size(); ++faceIndex) {
			for (int neighbor : topo.faceNeighborIndices[faceIndex]) {
				if (neighbor < 0) {
					throw std::runtime_error("IcosahedronTopology has boundary edge");
				}
			}
		}
		
        return topo;
    }

    std::vector<int> DefaultModuleOwningFaces(int moduleCount) const
    {
        if (moduleCount <= 0) {
            throw std::runtime_error("moduleCount must be > 0");
        }

        // Hardcoded physical camera placement on the icosahedron.
        // module 0 -> face 0
        // module 1 -> face 1
        static const std::vector<int> kOwningFaces = {
            0,
            1
        };

        if (moduleCount > static_cast<int>(kOwningFaces.size())) {
            throw std::runtime_error(
                "Requested more modules than hardcoded icosahedron face assignments");
        }

        return std::vector<int>(kOwningFaces.begin(),
                                kOwningFaces.begin() + moduleCount);
    }
};

} // namespace DASPi
