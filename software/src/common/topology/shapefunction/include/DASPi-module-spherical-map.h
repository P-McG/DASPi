// DASPi-module-spherical-map.h
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>
#include <iomanip>
#include <iostream>

#include <Eigen/Dense>

#include "DASPi-config.h"
#include "DASPi-globallinearspace.h"
#include "DASPi-icosahedronspherespace.h"
#include "DASPi-module-spherical-basis.h"

namespace DASPi {

template<class SphereSpaceType, std::size_t ModuleIndex, unsigned int FacetIndex>
class ModuleSphericalMap {
public:
    static_assert(IcosahedronSphereSpace_t<SphereSpaceType>);

    static constexpr std::size_t expectedFacetIndex_ =
        SphereSpaceType::template ModuleFaceIndex<ModuleIndex>();

    static_assert(
        expectedFacetIndex_ == FacetIndex,
        "ModuleIndex does not map to this FacetIndex"
    );

    static constexpr std::size_t regionCount_ = NUM_REGIONS;

    /*
     * Current destination space:
     *
     *   equirectangular global output using the same dimensions as the
     *   sensor frame.
     *
     * This keeps the downstream buffer sizes stable for the first global
     * spherical-map patch.
     */
    static constexpr std::uint32_t outputWidth_ =
        static_cast<std::uint32_t>(sensorWidthValue_);

    static constexpr std::uint32_t outputHeight_ =
        static_cast<std::uint32_t>(sensorHeightValue_);

    static constexpr std::size_t outputSizeValue_ =
        static_cast<std::size_t>(sensorWidthValue_) *
        static_cast<std::size_t>(sensorHeightValue_);

    static_assert(
        outputSizeValue_ <= std::numeric_limits<std::uint32_t>::max(),
        "ModuleSphericalMap output size does not fit in uint32_t"
    );

    static constexpr std::uint32_t outputSize_ =
        static_cast<std::uint32_t>(outputSizeValue_);

    /*
     * This value comes from the icosahedron topology table:
     *
     *   pixelsPerRadian = pixelsPerNormalToNormalAngle / normalToNormalAngle
     *
     * It is the current bridge from sensor pixel offsets to angular offsets.
     */
    static constexpr double pixelsPerRadian_ =
        DASPi::detail::IcosahedronTables::pixelsPerRadian_;

    using RegionMap =
        std::vector<std::uint32_t>;

    using RegionMaps =
        std::array<RegionMap, regionCount_>;

    RegionMaps regionGlobalIndices_{};

    template<class OverlapTopologyType>
    explicit ModuleSphericalMap(const OverlapTopologyType& topology)
    {
        static constexpr bool debugSphericalEdgeError_{true};
        static constexpr int debugEdgeSampleCount_{8};
        
        Build(topology);
    }

    constexpr std::uint32_t OutputWidth() const noexcept
    {
        return outputWidth_;
    }

    constexpr std::uint32_t OutputHeight() const noexcept
    {
        return outputHeight_;
    }

    constexpr std::uint32_t OutputSize() const noexcept
    {
        return outputSize_;
    }

    const RegionMap& Region(std::size_t regionIndex) const
    {
        return regionGlobalIndices_.at(regionIndex);
    }

private:
    template<class OverlapTopologyType>
    void Build(const OverlapTopologyType& topology)
    {
        const Eigen::Matrix3d Rcw =
            MakeModuleCameraRcw<
                SphereSpaceType,
                ModuleIndex,
                FacetIndex
            >();

        const auto& nonOverlapTopology =
            static_cast<
                const typename OverlapTopologyType::NonOverlapFacetTopology_t&
            >(topology);

        if constexpr (debugSphericalEdgeError_) {
            DebugPrintFaceEdgeProjectionError(
                nonOverlapTopology,
                Rcw
            );
        }

        AppendFromIndexLinear(
            0,
            *nonOverlapTopology.indexLinearMax_,
            nonOverlapTopology.size(),
            Rcw
        );

        for (std::size_t regionIndex = 1;
             regionIndex < regionCount_;
             ++regionIndex) {

            const std::size_t overlapIndex =
                regionIndex - 1;

            AppendFromIndexLinear(
                regionIndex,
                *topology.indexLinearMaxs_[overlapIndex],
                topology.size(overlapIndex),
                Rcw
            );
        }
    }
    
    static double ClampUnit(double value) noexcept
    {
        return std::clamp(value, -1.0, 1.0);
    }
    
    static double RadToDeg(double rad) noexcept
    {
        return rad * 180.0 / std::numbers::pi;
    }
    
    static Eigen::Vector3d SensorPointToCameraRay(double x, double y)
    {
        const double cx =
            static_cast<double>(sensorWidthValue_) * 0.5;
    
        const double cy =
            static_cast<double>(sensorHeightValue_) * 0.5;
    
        /*
         * Positive camera Y follows the existing camera model convention:
         *
         *   uv.y > cy -> ray.y > 0
         */
        const double dx =
            (x - cx) / pixelsPerRadian_;
    
        const double dy =
            (y - cy) / pixelsPerRadian_;
    
        const double radius =
            std::sqrt(dx * dx + dy * dy);
    
        if (radius < 1.0e-12) {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
    
        /*
         * Current equidistant angular model.
         *
         * This is the thing we are testing.
         */
        const double theta =
            radius;
    
        const double sinTheta =
            std::sin(theta);
    
        const double cosTheta =
            std::cos(theta);
    
        const double scale =
            sinTheta / radius;
    
        return Eigen::Vector3d(
            dx * scale,
            dy * scale,
            cosTheta
        ).normalized();
    }
    
    static Eigen::Vector3d SensorIndexToCameraRay(std::uint32_t sensorIndex)
    {
        const std::uint32_t x =
            sensorIndex % static_cast<std::uint32_t>(sensorWidthValue_);
    
        const std::uint32_t y =
            sensorIndex / static_cast<std::uint32_t>(sensorWidthValue_);
    
        if (y >= static_cast<std::uint32_t>(sensorHeightValue_)) {
            throw std::out_of_range(
                "ModuleSphericalMap: sensor index outside sensor frame"
            );
        }
    
        return SensorPointToCameraRay(
            static_cast<double>(x) + 0.5,
            static_cast<double>(y) + 0.5
        );
    }

    ////Test Version
    //static Eigen::Vector3d SensorIndexToCameraRay(std::uint32_t sensorIndex)
    //{
        //const std::uint32_t x =
            //sensorIndex % static_cast<std::uint32_t>(sensorWidthValue_);
    
        //const std::uint32_t y =
            //sensorIndex / static_cast<std::uint32_t>(sensorWidthValue_);
    
        //if (y >= static_cast<std::uint32_t>(sensorHeightValue_)) {
            //throw std::out_of_range(
                //"ModuleSphericalMap: sensor index outside sensor frame"
            //);
        //}
    
        //const double cx =
            //static_cast<double>(sensorWidthValue_) * 0.5;
    
        //const double cy =
            //static_cast<double>(sensorHeightValue_) * 0.5;
    
        ///*
         //* Flat triangle center-to-center spacing across a shared edge.
         //*/
        //constexpr double facetCenterToCenterPx =
            //static_cast<double>(sensorHeightValue_) *
            //std::numbers::sqrt3 / 3.0;
    
        ///*
         //* For gnomonic/pinhole:
         //*
         //*     tan(theta) = pixel_offset / focal_px
         //*
         //* so:
         //*
         //*     focal_px = pixel_offset / tan(theta)
         //*/
        //constexpr double cameraFocalPx =
            //facetCenterToCenterPx /
            //std::tan(DASPi::detail::IcosahedronTables::normalToNormalAngle_);
    
        //const double nx =
            //(static_cast<double>(x) + 0.5 - cx) / cameraFocalPx;
    
        //const double ny =
            //(static_cast<double>(y) + 0.5 - cy) / cameraFocalPx;
    
        //return Eigen::Vector3d(
            //nx,
            //ny,
            //1.0
        //).normalized();
    //}

    //static Eigen::Vector3d SensorIndexToCameraRay(std::uint32_t sensorIndex)
    //{
        //const std::uint32_t x =
            //sensorIndex % static_cast<std::uint32_t>(sensorWidthValue_);

        //const std::uint32_t y =
            //sensorIndex / static_cast<std::uint32_t>(sensorWidthValue_);

        //if (y >= static_cast<std::uint32_t>(sensorHeightValue_)) {
            //throw std::out_of_range(
                //"ModuleSphericalMap: sensor index outside sensor frame"
            //);
        //}

        //const double cx =
            //static_cast<double>(sensorWidthValue_) * 0.5;

        //const double cy =
            //static_cast<double>(sensorHeightValue_) * 0.5;

        ///*
         //* Positive camera Y follows the existing camera model convention:
         //*
         //*   uv.y > cy -> ray.y > 0
         //*
         //* If the output appears vertically flipped, this is the single place
         //* to change by negating dy.
         //*/
        //const double dx =
            //(static_cast<double>(x) + 0.5 - cx) / pixelsPerRadian_;

        //const double dy =
            //(static_cast<double>(y) + 0.5 - cy) / pixelsPerRadian_;

        //const double radius =
            //std::sqrt(dx * dx + dy * dy);

        //if (radius < 1.0e-12) {
            //return Eigen::Vector3d(0.0, 0.0, 1.0);
        //}

        ///*
         //* Equidistant angular model:
         //*
         //*   radius in normalized pixel space is theta in radians.
         //*
         //* This is intentionally simple for the first global spherical-map
         //* patch and uses the topology's pixels-per-radian scale.
         //*/
        //const double theta =
            //radius;

        //const double sinTheta =
            //std::sin(theta);

        //const double cosTheta =
            //std::cos(theta);

        //const double scale =
            //sinTheta / radius;

        //return Eigen::Vector3d(
            //dx * scale,
            //dy * scale,
            //cosTheta
        //).normalized();
    //}

    static std::uint32_t WorldRayToEquirectIndex(
        const Eigen::Vector3d& rayWorld)
    {
        const Eigen::Vector3d r =
            rayWorld.normalized();

        const double longitude =
            std::atan2(r.x(), r.z());

        const double latitude =
            std::asin(std::clamp(r.y(), -1.0, 1.0));

        double u =
            (longitude / (2.0 * std::numbers::pi) + 0.5) *
            static_cast<double>(outputWidth_);

        double v =
            (0.5 - latitude / std::numbers::pi) *
            static_cast<double>(outputHeight_);

        const auto clampPixel =
            [](double value, std::uint32_t limit) -> std::uint32_t
        {
            if (value < 0.0) {
                return 0;
            }

            const double maxValue =
                static_cast<double>(limit - 1);

            if (value > maxValue) {
                return limit - 1;
            }

            return static_cast<std::uint32_t>(value);
        };

        const std::uint32_t x =
            clampPixel(u, outputWidth_);

        const std::uint32_t y =
            clampPixel(v, outputHeight_);

        return y * outputWidth_ + x;
    }
    
    template<class NonOverlapTopologyType>
    static void DebugPrintFaceEdgeProjectionError(
        const NonOverlapTopologyType& flatTopology,
        const Eigen::Matrix3d& Rcw)
    {
        static bool printed = false;
    
        if (printed) {
            return;
        }
    
        printed = true;
    
        constexpr std::size_t N =
            SphereSpaceType::verticesPerFaceN_;
    
        const MeshTopology<N> mesh =
            MakeMeshTopologyFromSphereSpace<SphereSpaceType, N>();
    
        const RigData<N> rig =
            BuildRigDataFromTopology(mesh);
    
        if (FacetIndex >= rig.faces.size()) {
            throw std::runtime_error(
                "DebugPrintFaceEdgeProjectionError: FacetIndex out of range"
            );
        }
    
        constexpr std::size_t anchorFaceIndex =
            SphereSpaceType::moduleFaceIndices_[0];
    
        if (anchorFaceIndex >= rig.faces.size()) {
            throw std::runtime_error(
                "DebugPrintFaceEdgeProjectionError: anchorFaceIndex out of range"
            );
        }
    
        const RigFace<N>& face =
            rig.faces[FacetIndex];
    
        const RigFace<N>& anchorFace =
            rig.faces[anchorFaceIndex];
    
        /*
         * Make expected spherical face edges live in the same world frame as Rcw.
         *
         * MakeModuleCameraRcw() returns:
         *
         *     Rrig * Rface * Ralign * Rimg
         *
         * so the reference face vertices need Rrig too.
         */
        const Eigen::Matrix3d Rrig =
            RotationAligningAToB(
                anchorFace.lookDir,
                Eigen::Vector3d(0.0, 0.0, 1.0)
            );
    
        std::cout << "\n[SphericalEdgeDebug]"
                  << " module=" << ModuleIndex
                  << " facet=" << FacetIndex
                  << " pixelsPerRadian=" << std::setprecision(12)
                  << pixelsPerRadian_
                  << '\n';
    
        for (std::size_t localEdge = 0; localEdge < N; ++localEdge) {
            const std::size_t nextEdge =
                (localEdge + 1) % N;
    
            const auto& p0 =
                flatTopology.shapeDefiningPoints_[localEdge];
    
            const auto& p1 =
                flatTopology.shapeDefiningPoints_[nextEdge];
    
            const Eigen::Vector3d aWorld =
                (Rrig * rig.vertices[face.indices[localEdge]]).normalized();
    
            const Eigen::Vector3d bWorld =
                (Rrig * rig.vertices[face.indices[nextEdge]]).normalized();
    
            Eigen::Vector3d edgePlaneNormal =
                aWorld.cross(bWorld);
    
            const double edgePlaneNorm =
                edgePlaneNormal.norm();
    
            if (edgePlaneNorm < 1.0e-12) {
                throw std::runtime_error(
                    "DebugPrintFaceEdgeProjectionError: degenerate edge plane"
                );
            }
    
            edgePlaneNormal /= edgePlaneNorm;
    
            double maxAbsErrorDeg = 0.0;
    
            std::cout << "  edge=" << localEdge
                      << " flat=("
                      << p0.x() << "," << p0.y()
                      << ") -> ("
                      << p1.x() << "," << p1.y()
                      << ")\n";
    
            for (int sample = 0;
                 sample <= debugEdgeSampleCount_;
                 ++sample) {
    
                const double t =
                    static_cast<double>(sample) /
                    static_cast<double>(debugEdgeSampleCount_);
    
                const double sx =
                    (1.0 - t) * static_cast<double>(p0.x()) +
                    t * static_cast<double>(p1.x());
    
                const double sy =
                    (1.0 - t) * static_cast<double>(p0.y()) +
                    t * static_cast<double>(p1.y());
    
                const Eigen::Vector3d rayCamera =
                    SensorPointToCameraRay(sx, sy);
    
                const Eigen::Vector3d rayWorld =
                    (Rcw * rayCamera).normalized();
    
                const double signedErrorRad =
                    std::asin(
                        ClampUnit(
                            rayWorld.dot(edgePlaneNormal)
                        )
                    );
    
                const double signedErrorDeg =
                    RadToDeg(signedErrorRad);
    
                maxAbsErrorDeg =
                    std::max(
                        maxAbsErrorDeg,
                        std::abs(signedErrorDeg)
                    );
    
                std::cout << "    t=" << std::fixed << std::setprecision(3)
                          << t
                          << " sensor=("
                          << std::setprecision(3)
                          << sx << "," << sy
                          << ") edgeErrorDeg="
                          << std::setprecision(6)
                          << signedErrorDeg
                          << '\n';
            }
    
            std::cout << "    maxAbsErrorDeg="
                      << std::setprecision(6)
                      << maxAbsErrorDeg
                      << '\n';
        }
    
        std::cout << std::endl;
    }

    static std::uint32_t SensorIndexToOutputIndex(
        std::uint32_t sensorIndex,
        const Eigen::Matrix3d& Rcw)
    {
        const Eigen::Vector3d rayCamera =
            SensorIndexToCameraRay(sensorIndex);

        const Eigen::Vector3d rayWorld =
            (Rcw * rayCamera).normalized();

        return WorldRayToEquirectIndex(rayWorld);
    }

    template<class IndexLinear>
    void AppendFromIndexLinear(
        std::size_t regionIndex,
        const IndexLinear& indexLinear,
        std::size_t validCount,
        const Eigen::Matrix3d& Rcw)
    {
        if (regionIndex >= regionCount_) {
            throw std::out_of_range(
                "ModuleSphericalMap: regionIndex out of range"
            );
        }

        if (validCount > indexLinear.size()) {
            throw std::runtime_error(
                "ModuleSphericalMap: validCount exceeds indexLinear size"
            );
        }

        auto& dst =
            regionGlobalIndices_[regionIndex];

        dst.clear();
        dst.reserve(validCount);

        for (std::size_t i = 0; i < validCount; ++i) {
            const std::uint32_t sensorIndex =
                static_cast<std::uint32_t>(indexLinear[i].value());

            const std::uint32_t outputIndex =
                SensorIndexToOutputIndex(
                    sensorIndex,
                    Rcw
                );

            if (outputIndex >= outputSize_) {
                throw std::runtime_error(
                    "ModuleSphericalMap: outputIndex outside outputSize"
                );
            }

            dst.push_back(outputIndex);
        }
    }
};

} // namespace DASPi
