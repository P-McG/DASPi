// DASPi-module-spherical-basis.h
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <numbers>
#include <string>

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

inline Eigen::Matrix3d RotationAligningAToB(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
    const Eigen::Vector3d an = a.normalized();
    const Eigen::Vector3d bn = b.normalized();

    Eigen::Quaterniond q;
    q.setFromTwoVectors(an, bn);

    return q.normalized().toRotationMatrix();
}


template<class SphereSpace, std::size_t N>
inline MeshTopology<N> MakeMeshTopologyFromSphereSpace()
{
    static_assert(DASPi::IcosahedronSphereSpace_t<SphereSpace>);

    static_assert(
        N == SphereSpace::verticesPerFaceN_,
        "N must match SphereSpace::verticesPerFaceN_"
    );

    using Facet0SpaceType =
        typename SphereSpace::template FacetSpace_t<0>;

    using Tables =
        DASPi::detail::IcosahedronTables;

    static_assert(
        SphereSpace::totalFacetsN_ == Tables::facetsN_,
        "SphereSpace total face count must match IcosahedronTables"
    );

    static_assert(
        Facet0SpaceType::verticesN_ == Tables::verticesN_,
        "Facet0SpaceType vertex count must match IcosahedronTables"
    );

    static_assert(
        Facet0SpaceType::edgesN_ == Tables::edgesN_,
        "Facet0SpaceType edge count must match IcosahedronTables"
    );

    MeshTopology<N> topo;

    const Eigen::Matrix3d Rspace =
        ToEigenMatrix3d(Facet0SpaceType::ImageTransformMatrix);

    topo.vertices.reserve(Tables::verticesN_);

    for (const auto& vertex : Tables::vertices_) {
        const Eigen::Vector3d v(
            vertex.x_,
            vertex.y_,
            vertex.z_
        );

        topo.vertices.push_back(Rspace * v);
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

    if (topo.edges.size() != Facet0SpaceType::edgesN_) {
        throw std::runtime_error(
            "MakeMeshTopologyFromSphereSpace expected " +
            std::to_string(Facet0SpaceType::edgesN_) +
            " edges, got " +
            std::to_string(topo.edges.size())
        );
    }

    for (std::size_t faceIndex = 0;
         faceIndex < topo.faceNeighborIndices.size();
         ++faceIndex) {

        for (int neighbor : topo.faceNeighborIndices[faceIndex]) {
            if (neighbor < 0) {
                throw std::runtime_error(
                    "MakeMeshTopologyFromSphereSpace has boundary edge"
                );
            }
        }
    }

    return topo;
}

template<std::size_t N>
inline RigData<N> BuildRigDataFromTopology(const MeshTopology<N>& topo)
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
	    deg * std::numbers::pi / 180.0;

    return Eigen::AngleAxisd(
        rad,
        Eigen::Vector3d::UnitZ()
    ).toRotationMatrix();
}

inline Eigen::Matrix3d CameraLocalPitchDeg(double deg)
{
    const double rad =
        deg * std::numbers::pi / 180.0;

    /*
     * Local camera +X axis.
     *
     * This tilts the camera forward/backward in its own frame.
     */
    return Eigen::AngleAxisd(
        rad,
        Eigen::Vector3d::UnitX()
    ).toRotationMatrix();
}

inline Eigen::Matrix3d CameraLocalYawDeg(double deg)
{
    const double rad =
        deg * std::numbers::pi / 180.0;

    /*
     * Local camera +Y axis.
     *
     * This pans the camera left/right in its own frame.
     */
    return Eigen::AngleAxisd(
        rad,
        Eigen::Vector3d::UnitY()
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

template<std::size_t ModuleIndex>
inline constexpr double CameraLocalRollCorrectionDegForModule()
{
    if constexpr (ModuleIndex == 0) {
        return 0.0;      // keep module 0 as the reference
    } else if constexpr (ModuleIndex == 1) {
        return 0.0/*-12.0*/;      // tune this: try +/- 0.25, +/- 0.5, +/- 1.0
    } else {
        return 0.0;
    }
}

template<std::size_t ModuleIndex>
inline constexpr double CameraLocalYawCorrectionDegForModule()
{
    if constexpr (ModuleIndex == 0) {
        return 0.0;
    } else if constexpr (ModuleIndex == 1) {
        return 0.0;
    } else {
        return 0.0;
    }
}

template<std::size_t ModuleIndex>
inline constexpr double CameraLocalPitchCorrectionDegForModule()
{
    if constexpr (ModuleIndex == 0) {
        return 0.0;
    } else if constexpr (ModuleIndex == 1) {
        return 0.0;
    } else {
        return 0.0;
    }
}

//template<std::size_t ModuleIndex>
//inline constexpr double CameraLocalRollCorrectionDegForModule()
//{
    //if constexpr (ModuleIndex == 0) {
        //return 0.0;
    //} else if constexpr (ModuleIndex == 1) {
        //return 0.0;
    //} else {
        //return 0.0;
    //}
//}

template<std::size_t ModuleIndex>
inline Eigen::Matrix3d CameraLocalExtrinsicCorrection()
{
    const Eigen::Matrix3d Ryaw =
        CameraLocalYawDeg(
            CameraLocalYawCorrectionDegForModule<ModuleIndex>()
        );

    const Eigen::Matrix3d Rpitch =
        CameraLocalPitchDeg(
            CameraLocalPitchCorrectionDegForModule<ModuleIndex>()
        );

    const Eigen::Matrix3d Rroll =
        CameraRollDeg(
            CameraLocalRollCorrectionDegForModule<ModuleIndex>()
        );

    /*
     * Local-frame correction.
     *
     * This matrix is post-multiplied onto Rcw, so the correction happens
     * in the camera's own local axes:
     *
     *   +X = camera right
     *   +Y = camera up
     *   +Z = camera forward
     */
    return Ryaw * Rpitch * Rroll;
}

inline Eigen::Matrix3d CameraModelToRigAlignment()
{
    return Eigen::Matrix3d::Identity();
}

template<std::size_t N>
inline std::array<std::size_t, 2> FindSharedEdgeVertices(
    const RigFace<N>& a,
    const RigFace<N>& b)
{
    static_assert(N >= 2, "Faces must have at least 2 vertices");

    std::array<std::size_t, 2> shared{};
    std::size_t count = 0;

    for (std::size_t va : a.indices) {
        for (std::size_t vb : b.indices) {
            if (va == vb) {
                if (count >= 2) {
                    throw std::runtime_error("Faces share more than one edge");
                }

                shared[count++] = va;
            }
        }
    }

    if (count != 2) {
        throw std::runtime_error("Faces do not share exactly one edge");
    }

    return shared;
}

template<std::size_t N>
inline Eigen::Vector3d MakeFaceSeamTangent(
    const RigFace<N>& face,
    const std::vector<Eigen::Vector3d>& vertices,
    std::size_t vi0,
    std::size_t vi1)
{
    if (vi0 >= vertices.size() || vi1 >= vertices.size()) {
        throw std::runtime_error("Shared edge vertex index out of range");
    }

    Eigen::Vector3d edgeWorld =
        (vertices[vi1] - vertices[vi0]).normalized();

    const Eigen::Vector3d forward =
        face.lookDir.normalized();

    edgeWorld -= forward * forward.dot(edgeWorld);

    const double norm = edgeWorld.norm();

    if (norm < 1.0e-12) {
        throw std::runtime_error("Degenerate seam tangent");
    }

    return edgeWorld / norm;
}


template<
    class SphereSpaceType,
    std::size_t ModuleIndex,
    unsigned int FacetIndex
>
inline Eigen::Matrix3d MakeModuleCameraRcw()
{
    static_assert(DASPi::IcosahedronSphereSpace_t<SphereSpaceType>);

    static_assert(
        SphereSpaceType::template ModuleFaceIndex<ModuleIndex>() ==
        FacetIndex,
        "ModuleIndex does not map to FacetIndex"
    );

    constexpr std::size_t N =
        SphereSpaceType::verticesPerFaceN_;

    constexpr std::size_t kModuleCount =
        SphereSpaceType::moduleFacesN_;

    static_assert(ModuleIndex < kModuleCount);

    const MeshTopology<N> topo =
        MakeMeshTopologyFromSphereSpace<SphereSpaceType, N>();

    const RigData<N> rig =
        BuildRigDataFromTopology(topo);

    const auto& faces =
        rig.faces;

    const auto& vertices =
        rig.vertices;

    constexpr std::size_t anchorFaceIndex =
        SphereSpaceType::moduleFaceIndices_[0];

    if (anchorFaceIndex >= faces.size()) {
        throw std::runtime_error(
            "Anchor face index outside rig face range"
        );
    }

    if (FacetIndex >= faces.size()) {
        throw std::runtime_error(
            "FacetIndex outside rig face range"
        );
    }

    const RigFace<N>& anchorFace =
        faces[anchorFaceIndex];

    const Eigen::Matrix3d Rrig =
        RotationAligningAToB(
            anchorFace.lookDir,
            Eigen::Vector3d(0.0, 0.0, 1.0)
        );

    const Eigen::Matrix3d Ralign =
        CameraModelToRigAlignment();

    Eigen::Matrix3d Rface =
        Eigen::Matrix3d::Identity();

    if constexpr (kModuleCount == 1) {
        const RigFace<N>& face =
            faces[FacetIndex];

        Rface =
            MakeCameraRcwFromFace<N>(
                face,
                vertices
            );
    } else if constexpr (kModuleCount == 2) {
        constexpr std::size_t faceIndex0 =
            SphereSpaceType::moduleFaceIndices_[0];

        constexpr std::size_t faceIndex1 =
            SphereSpaceType::moduleFaceIndices_[1];

        const RigFace<N>& face0 =
            faces[faceIndex0];

        const RigFace<N>& face1 =
            faces[faceIndex1];

        const auto sharedEdge =
            FindSharedEdgeVertices<N>(
                face0,
                face1
            );

        Eigen::Vector3d seam0 =
            MakeFaceSeamTangent<N>(
                face0,
                vertices,
                sharedEdge[0],
                sharedEdge[1]
            );

        Eigen::Vector3d seam1 =
            MakeFaceSeamTangent<N>(
                face1,
                vertices,
                sharedEdge[0],
                sharedEdge[1]
            );

        if (seam0.dot(seam1) < 0.0) {
            seam1 = -seam1;
        }

        if constexpr (ModuleIndex == 0) {
            Rface =
                MakeCameraRcwFromForwardAndUpHint(
                    face0.lookDir,
                    seam0
                );
        } else {
            Rface =
                MakeCameraRcwFromForwardAndUpHint(
                    face1.lookDir,
                    seam1
                );
        }
    } else {
        const RigFace<N>& face =
            faces[FacetIndex];

        Rface =
            MakeCameraRcwFromFace<N>(
                face,
                vertices
            );
    }

	const Eigen::Matrix3d Rimg =
	    CameraRollDeg(
	        CameraLocalRollDegForModule<ModuleIndex>()
	    );
	
	const Eigen::Matrix3d Rcal =
	    CameraLocalExtrinsicCorrection<ModuleIndex>();
	
	return Rrig * Rface * Ralign * Rimg * Rcal;
}

} // namespace DASPi
