// DASPi-module-spherical-basis.h
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "DASPi-icosahedronspace.h"
#include "DASPi-icosahedronspherespace.h"
#include "DASPi-mesh_topology.h"
#include "DASPi-rig-data.h"
#include "DASPi-rotation.h"

namespace DASPi {

inline Eigen::Matrix3d ToEigenMatrix3d(const Matrix3dData& m)
{
    Eigen::Matrix3d out;

    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            out(
                static_cast<Eigen::Index>(row),
                static_cast<Eigen::Index>(col)
            ) = m[row][col];
        }
    }

    return out;
}

template<std::size_t N>
inline MeshTopology<N> MakeMeshTopologyFromSphereSpaceTables()
{
    using Tables = DASPi::detail::IcosahedronTables;

    static_assert(N == Tables::verticesPerFaceN_);

    MeshTopology<N> topo;

    topo.vertices.reserve(Tables::verticesN_);

    /*
     * Same base vertex table used by the current ComputeModule path.
     * Keep this unrotated here for now; the module basis applies the
     * same global-space alignment separately.
     */
    for (const auto& vertex : Tables::vertices_) {
        topo.vertices.emplace_back(
            vertex.x_,
            vertex.y_,
            vertex.z_
        );
    }

    topo.faces.reserve(Tables::facetsN_);

    for (const auto& sourceFace : Tables::facets_) {
        std::array<std::size_t, N> face{};

        for (std::size_t i = 0; i < N; ++i) {
            face[i] = sourceFace[i];
        }

        topo.faces.push_back(face);
    }

    BuildTopologyAdjacency(topo);

    return topo;
}

template<std::size_t N>
inline RigData<N> BuildRigDataFromMeshTopology(const MeshTopology<N>& topo)
{
    RigData<N> rig;

    rig.vertices.reserve(topo.vertices.size());

    for (const auto& v : topo.vertices) {
        rig.vertices.push_back(v.normalized());
    }

    rig.faces.reserve(topo.faces.size());

    for (const auto& faceIndices : topo.faces) {
        RigFace<N> face;
        face.indices = faceIndices;

        Eigen::Vector3d c =
            Eigen::Vector3d::Zero();

        for (const std::size_t vi : faceIndices) {
            c += rig.vertices.at(vi);
        }

        face.normal = c.normalized();
        face.lookDir = face.normal;

        rig.faces.push_back(face);
    }

    return rig;
}

template<std::size_t N>
inline Eigen::Matrix3d MakeCameraRcwFromFace(
    const RigFace<N>& face,
    const std::vector<Eigen::Vector3d>& vertices)
{
    const Eigen::Vector3d v0 =
        vertices[face.indices[0]].normalized();

    const Eigen::Vector3d v1 =
        vertices[face.indices[1]].normalized();

    const Eigen::Vector3d v2 =
        vertices[face.indices[2]].normalized();

    const Eigen::Vector3d forward =
        face.lookDir.normalized();

    Eigen::Vector3d edge =
        v1 - v0;

    edge -= forward * edge.dot(forward);

    if (edge.norm() < 1.0e-12) {
        edge = v2 - v0;
        edge -= forward * edge.dot(forward);
    }

    if (edge.norm() < 1.0e-12) {
        throw std::runtime_error("Failed to construct face edge direction");
    }

    edge.normalize();

    Eigen::Vector3d up =
        edge;

    Eigen::Vector3d right =
        up.cross(forward).normalized();

    up =
        forward.cross(right).normalized();

    Eigen::Matrix3d Rcw;
    Rcw.col(0) = right;
    Rcw.col(1) = up;
    Rcw.col(2) = forward;

    return Rcw;
}

inline Eigen::Matrix3d MakeCameraRcwFromForwardAndUpHint(
    const Eigen::Vector3d& forward,
    const Eigen::Vector3d& upHint)
{
    const Eigen::Vector3d z =
        forward.normalized();

    Eigen::Vector3d y =
        upHint - z * z.dot(upHint);

    if (y.norm() < 1.0e-12) {
        throw std::runtime_error("Degenerate up hint");
    }

    y.normalize();

    Eigen::Vector3d x =
        y.cross(z).normalized();

    y =
        z.cross(x).normalized();

    Eigen::Matrix3d Rcw;
    Rcw.col(0) = x;
    Rcw.col(1) = y;
    Rcw.col(2) = z;

    return Rcw;
}

inline Eigen::Matrix3d CameraRollDeg(double deg)
{
    const double rad =
        deg * M_PI / 180.0;

    return Eigen::AngleAxisd(
        rad,
        Eigen::Vector3d::UnitZ()
    ).toRotationMatrix();
}

template<std::size_t ModuleIndex>
inline constexpr double CameraLocalRollDegForModule()
{
    if constexpr (ModuleIndex == 0) {
        return 0.0;
    } else if constexpr (ModuleIndex == 1) {
        return 180.0;
    } else {
        return 0.0;
    }
}

inline Eigen::Matrix3d CameraModelToRigAlignment()
{
    return Eigen::Matrix3d::Identity();
}

template<
    class SphereSpaceType,
    std::size_t ModuleIndex,
    unsigned int FacetIndex
>
inline Eigen::Matrix3d MakeModuleCameraRcw()
{
    static_assert(IcosahedronSphereSpace_t<SphereSpaceType>);
    static_assert(
        SphereSpaceType::template ModuleFaceIndex<ModuleIndex>() ==
        FacetIndex
    );

    constexpr std::size_t N =
        SphereSpaceType::verticesPerFaceN_;

    const MeshTopology<N> topo =
        MakeMeshTopologyFromSphereSpaceTables<N>();

    const RigData<N> rig =
        BuildRigDataFromMeshTopology(topo);

    if (FacetIndex >= rig.faces.size()) {
        throw std::runtime_error("FacetIndex out of rig face range");
    }

    constexpr std::size_t anchorFaceIndex =
        SphereSpaceType::moduleFaceIndices_[0];

    const RigFace<N>& anchorFace =
        rig.faces.at(anchorFaceIndex);

    const Eigen::Matrix3d Rrig =
        RotationAligningAToB(
            anchorFace.lookDir,
            Eigen::Vector3d(0.0, 0.0, 1.0)
        );

    const RigFace<N>& face =
        rig.faces.at(FacetIndex);

    const Eigen::Matrix3d Rface =
        MakeCameraRcwFromFace<N>(
            face,
            rig.vertices
        );

    const Eigen::Matrix3d Ralign =
        CameraModelToRigAlignment();

    const Eigen::Matrix3d Rimg =
        CameraRollDeg(
            CameraLocalRollDegForModule<ModuleIndex>()
        );

    return Rrig * Rface * Ralign * Rimg;
}

} // namespace DASPi
