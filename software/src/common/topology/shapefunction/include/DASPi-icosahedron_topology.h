// DASPi-icosahedron_topology.h
#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>
#include <numbers>

#include "DASPi-global_axes.h"
#include "DASPi-sphere_projection.h"
#include "DASPi-sphere_pixel.h"
#include "DASPi-icosahedronspace.h"
#include "DASPi-mesh_topology.h"

namespace DASPi {

template<IcosahedronSpace_t Space>
class IcosahedronTopology {
public:
    
    static inline constexpr double kPi{std::numbers::pi};
    static inline constexpr double phi{ std::numbers::phi };
             
    MeshTopology<3> Make() const
    {
        MeshTopology<3> topo;

        topo.vertices = {
            {-1,  phi, 0}, { 1,  phi, 0}, {-1, -phi, 0}, { 1, -phi, 0},
            {0, -1,  phi}, {0,  1,  phi}, {0, -1, -phi}, {0,  1, -phi},
            { phi, 0, -1}, { phi, 0,  1}, {-phi, 0, -1}, {-phi, 0,  1},
        };

        const Eigen::Matrix3d R =
            Eigen::AngleAxisd(kPi / 5.0, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(kPi / 7.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

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
    
    static constexpr MaskOrientationData MaskOrientation(double radius, double cosValue, double sinValue){
        return MaskOrientationData{
	    +1 * static_cast<long>(radius * sinValue),
	    +1 * static_cast<long>(radius * cosValue)};
	};

    const GlobalAxes& Axes() const
    {
        return axes_;
    }

    const SphereProjection& Projection() const
    {
        return projection_;
    }

    SpherePixel RayToSpherePixel(const Eigen::Vector3d& rayGlobal) const
    {
        const Eigen::Vector3d r = rayGlobal.normalized();

        const double gx = r.dot(axes_.longitudeZero.normalized());
        const double gy = r.dot(axes_.longitudePositive.normalized());
        const double gz = r.dot(axes_.north.normalized());

        const double lon = std::atan2(gy, gx);
        const double lat = std::asin(std::clamp(gz, -1.0, 1.0));

        const double u = (lon + kPi) / (2.0 * kPi);
        const double v = (kPi * 0.5 - lat) / kPi;

        int x = static_cast<int>(u * static_cast<double>(projection_.width));
        int y = static_cast<int>(v * static_cast<double>(projection_.height));

        x = ((x % projection_.width) + projection_.width) % projection_.width;
        y = std::clamp(y, 0, projection_.height - 1);

        return {x, y};
    }

    std::vector<int> DefaultModuleOwningFaces(int moduleCount) const
    {
        if (moduleCount <= 0) {
            throw std::runtime_error("moduleCount must be > 0");
        }

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

private:
    GlobalAxes axes_;
    SphereProjection projection_;
};


} // namespace DASPi
