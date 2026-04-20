// DASPi-mesh_topology.h
#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
namespace DASPi{
	
template <std::size_t N>
struct MeshTopology {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<std::size_t, N>> faces;
};
};//namespace DASPi
