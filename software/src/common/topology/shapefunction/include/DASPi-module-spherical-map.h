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
#include "DASPi-sphere-map-wire.h"

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
    static constexpr double cameraFocalPx_ =
        DASPi::detail::IcosahedronTables::cameraFocalPx_;
        
    static constexpr bool debugSphericalEdgeError_{false};
    static constexpr int debugEdgeSampleCount_{8};

    using RegionMap =
        std::vector<std::uint32_t>;

    using RegionMaps =
        std::array<RegionMap, regionCount_>;

    RegionMaps regionGlobalIndices_{};

    template<class OverlapTopologyType>
    explicit ModuleSphericalMap(const OverlapTopologyType& topology)
    {      
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

    //static Eigen::Vector3d SensorPointToCameraRay_Gnomonic(double x, double y)
    //{
        //const double cx =
            //static_cast<double>(sensorWidthValue_) * 0.5;
    
        //const double cy =
            //static_cast<double>(sensorHeightValue_) * 0.5;
    
        ///*
         //* Actual current flat triangle radius from the logged points:
         //*
         //* center = (728, 544)
         //* vertex = (1199, 544)
         //* radius = 471 px
         //*
         //* This is 0.75 * sensorHeightValue_ * sqrt(3) / 3.
         //*/
        //constexpr double flatTriangleRadiusPx =
            //0.75 *
            //static_cast<double>(sensorHeightValue_) *
            //std::numbers::sqrt3 / 3.0;
    
        ///*
         //* Icosahedron face-center to vertex angle:
         //*
         //* For a regular icosahedron spherical face this is about:
         //* 37.3773681406 deg.
         //*/
        //constexpr double faceCenterToVertexAngleRad =
            //0.6523581397843682;
    
        ///*
         //* Gnomonic/pinhole:
         //*
         //*     tan(theta) = radius_px / focal_px
         //*/
        //constexpr double cameraFocalPx =
            //flatTriangleRadiusPx /
            //std::tan(faceCenterToVertexAngleRad);
    
        //const double nx =
            //(x - cx) / cameraFocalPx;
    
        //const double ny =
            //(y - cy) / cameraFocalPx;
    
        //return Eigen::Vector3d(
            //nx,
            //ny,
            //1.0
        //).normalized();
    //}

    //static Eigen::Vector3d SensorPointToCameraRay(double x, double y)
    //{
        //const double cx =
            //static_cast<double>(sensorWidthValue_) * 0.5;
    
        //const double cy =
            //static_cast<double>(sensorHeightValue_) * 0.5;
    
        ///*
         //* Positive camera Y follows the existing camera model convention:
         //*
         //*   uv.y > cy -> ray.y > 0
         //*/
        //const double dx =
            //(x - cx) / cameraFocalPx_;
    
        //const double dy =
            //(y - cy) / cameraFocalPx_;
    
        //const double radius =
            //std::sqrt(dx * dx + dy * dy);
    
        //if (radius < 1.0e-12) {
            //return Eigen::Vector3d(0.0, 0.0, 1.0);
        //}
    
        ///*
         //* Current equidistant angular model.
         //*
         //* This is the thing we are testing.
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
    
 static constexpr bool useGnomonicProjection_{true};

    static Eigen::Vector3d SensorPointToCameraRay_Equidistant(double x, double y)
    {
        const double cx =
            static_cast<double>(sensorWidthValue_) * 0.5;
    
        const double cy =
            static_cast<double>(sensorHeightValue_) * 0.5;
    
        const double dx =
            (x - cx) / cameraFocalPx_;
    
        const double dy =
            (y - cy) / cameraFocalPx_;
    
        const double radius =
            std::sqrt(dx * dx + dy * dy);
    
        if (radius < 1.0e-12) {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
    
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
    
    static Eigen::Vector3d SensorPointToCameraRay_Gnomonic(double x, double y)
    {
        const double cx =
            static_cast<double>(sensorWidthValue_) * 0.5;
    
        const double cy =
            static_cast<double>(sensorHeightValue_) * 0.5;
    
        constexpr double flatTriangleRadiusPx =
            0.75 *
            static_cast<double>(sensorHeightValue_) *
            std::numbers::sqrt3 / 3.0;
    
        constexpr double faceCenterToVertexAngleRad =
            0.6523581397843682;
    
        constexpr double cameraFocalPx =
            flatTriangleRadiusPx /
            std::tan(faceCenterToVertexAngleRad);
    
        const double nx =
            (x - cx) / cameraFocalPx;
    
        const double ny =
            (y - cy) / cameraFocalPx;
    
        return Eigen::Vector3d(
            nx,
            ny,
            1.0
        ).normalized();
    }
    
    static Eigen::Vector3d SensorPointToCameraRay(double x, double y)
    {
        const double cx =
            static_cast<double>(sensorWidthValue_) * 0.5;
    
        const double cy =
            static_cast<double>(sensorHeightValue_) * 0.5;
    
        const double nx =
            (x - cx) / cameraFocalPx_;
    
        const double ny =
            (y - cy) / cameraFocalPx_;
    
        return Eigen::Vector3d(
            nx,
            ny,
            1.0
        ).normalized();
    }
    
    //static Eigen::Vector3d SensorPointToCameraRay(double x, double y)
    //{
        //if constexpr (useGnomonicProjection_) {
            //return SensorPointToCameraRay_Gnomonic(x, y);
        //} else {
            //return SensorPointToCameraRay_Equidistant(x, y);
        //}
    //}

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
            //(static_cast<double>(x) + 0.5 - cx) / cameraFocalPx_;

        //const double dy =
            //(static_cast<double>(y) + 0.5 - cy) / cameraFocalPx_;

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
    
    static double PlaneAngleDeg(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b)
    {
        const double dot =
            std::abs(
                std::clamp(
                    a.normalized().dot(b.normalized()),
                    -1.0,
                    1.0
                )
            );
    
        return RadToDeg(std::acos(dot));
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
    
        const Eigen::Matrix3d Rrig =
            RotationAligningAToB(
                anchorFace.lookDir,
                Eigen::Vector3d(0.0, 0.0, 1.0)
            );
    
        std::array<Eigen::Vector3d, N> sphericalEdgePlanes{};
    
        for (std::size_t edge = 0; edge < N; ++edge) {
            const std::size_t next =
                (edge + 1) % N;
    
            const Eigen::Vector3d aWorld =
                (Rrig * rig.vertices[face.indices[edge]]).normalized();
    
            const Eigen::Vector3d bWorld =
                (Rrig * rig.vertices[face.indices[next]]).normalized();
    
            Eigen::Vector3d plane =
                aWorld.cross(bWorld);
    
            const double norm =
                plane.norm();
    
            if (norm < 1.0e-12) {
                throw std::runtime_error(
                    "DebugPrintFaceEdgeProjectionError: degenerate spherical edge plane"
                );
            }
    
            sphericalEdgePlanes[edge] =
                plane / norm;
        }
    
        std::cout << "\n[SphericalEdgeDebug]"
                  << " module=" << ModuleIndex
                  << " facet=" << FacetIndex
                  << " pixelsPerRadian=" << std::setprecision(12)
                  << cameraFocalPx_
                  << '\n';
    
        for (std::size_t flatEdge = 0; flatEdge < N; ++flatEdge) {
            const std::size_t nextFlatEdge =
                (flatEdge + 1) % N;
    
            const auto& p0 =
                flatTopology.shapeDefiningPoints_[flatEdge];
    
            const auto& p1 =
                flatTopology.shapeDefiningPoints_[nextFlatEdge];
    
            const double x0 =
                static_cast<double>(p0.x());
    
            const double y0 =
                static_cast<double>(p0.y());
    
            const double x1 =
                static_cast<double>(p1.x());
    
            const double y1 =
                static_cast<double>(p1.y());
    
            //const Eigen::Vector3d ray0 =
                //(Rcw * SensorPointToCameraRay(x0, y0)).normalized();
    
            //const Eigen::Vector3d ray1 =
                //(Rcw * SensorPointToCameraRay(x1, y1)).normalized();
                
            const Eigen::Vector3d ray0 =
                (Rcw * SensorPointToCameraRay_Gnomonic(x0, y0)).normalized();
            
            const Eigen::Vector3d ray1 =
                (Rcw * SensorPointToCameraRay_Gnomonic(x1, y1)).normalized();
    
            Eigen::Vector3d measuredPlane =
                ray0.cross(ray1);
    
            const double measuredPlaneNorm =
                measuredPlane.norm();
    
            if (measuredPlaneNorm < 1.0e-12) {
                throw std::runtime_error(
                    "DebugPrintFaceEdgeProjectionError: degenerate measured edge plane"
                );
            }
    
            measuredPlane /= measuredPlaneNorm;
    
            std::size_t bestSphericalEdge = 0;
            double bestPlaneAngleDeg =
                std::numeric_limits<double>::infinity();
    
            for (std::size_t sphericalEdge = 0;
                 sphericalEdge < N;
                 ++sphericalEdge) {
    
                const double angleDeg =
                    PlaneAngleDeg(
                        measuredPlane,
                        sphericalEdgePlanes[sphericalEdge]
                    );
    
                if (angleDeg < bestPlaneAngleDeg) {
                    bestPlaneAngleDeg = angleDeg;
                    bestSphericalEdge = sphericalEdge;
                }
            }
    
            std::cout << "  flatEdge=" << flatEdge
                      << " flat=("
                      << p0.x() << "," << p0.y()
                      << ") -> ("
                      << p1.x() << "," << p1.y()
                      << ")"
                      << " bestSphericalEdge=" << bestSphericalEdge
                      << " planeAngleDeg="
                      << std::setprecision(6)
                      << bestPlaneAngleDeg
                      << '\n';
    
            double maxMeasuredBowDeg = 0.0;
            double maxBestEdgeErrorDeg = 0.0;
    
            for (int sample = 0;
                 sample <= ModuleSphericalMap::debugEdgeSampleCount_;
                 ++sample) {
    
                const double t =
                    static_cast<double>(sample) /
                    static_cast<double>(
                        ModuleSphericalMap::debugEdgeSampleCount_
                    );
    
                const double sx =
                    (1.0 - t) * x0 + t * x1;
    
                const double sy =
                    (1.0 - t) * y0 + t * y1;
    
                //const Eigen::Vector3d rayWorld =
                    //(Rcw * SensorPointToCameraRay(sx, sy)).normalized();
                const Eigen::Vector3d rayWorld =
                    (Rcw * SensorPointToCameraRay_Gnomonic(sx, sy)).normalized();
    
                const double measuredBowDeg =
                    RadToDeg(
                        std::asin(
                            ClampUnit(
                                rayWorld.dot(measuredPlane)
                            )
                        )
                    );
    
                const double bestEdgeErrorDeg =
                    RadToDeg(
                        std::asin(
                            ClampUnit(
                                rayWorld.dot(
                                    sphericalEdgePlanes[bestSphericalEdge]
                                )
                            )
                        )
                    );
    
                maxMeasuredBowDeg =
                    std::max(
                        maxMeasuredBowDeg,
                        std::abs(measuredBowDeg)
                    );
    
                maxBestEdgeErrorDeg =
                    std::max(
                        maxBestEdgeErrorDeg,
                        std::abs(bestEdgeErrorDeg)
                    );
    
                std::cout << "    t=" << std::fixed << std::setprecision(3)
                          << t
                          << " sensor=("
                          << sx << "," << sy
                          << ") measuredBowDeg="
                          << std::setprecision(6)
                          << measuredBowDeg
                          << " bestEdgeErrorDeg="
                          << bestEdgeErrorDeg
                          << '\n';
            }
    
            std::cout << "    maxMeasuredBowDeg="
                      << std::setprecision(6)
                      << maxMeasuredBowDeg
                      << " maxBestEdgeErrorDeg="
                      << maxBestEdgeErrorDeg
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
    
    static SphereMapBayerChannel SensorIndexToBayerChannel(
        std::uint32_t sensorIndex)
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
    
        /*
         * Current mosaic convention used by Region0 WB:
         *
         *   even row, even col = Blue
         *   even row, odd  col = Green
         *   odd  row, even col = Green
         *   odd  row, odd  col = Red
         */
        const bool evenRow =
            (y & 1u) == 0u;
    
        const bool evenCol =
            (x & 1u) == 0u;
    
        if (evenRow) {
            return evenCol
                ? SphereMapBayerChannel::Blue
                : SphereMapBayerChannel::Green;
        }
    
        return evenCol
            ? SphereMapBayerChannel::Green
            : SphereMapBayerChannel::Red;
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

            const SphereMapBayerChannel channel =
                SensorIndexToBayerChannel(sensorIndex);
            
            const std::uint32_t packedEntry =
                PackSphereMapEntry(
                    outputIndex,
                    channel
                );
            
            dst.push_back(packedEntry);
        }
    }
};

} // namespace DASPi
