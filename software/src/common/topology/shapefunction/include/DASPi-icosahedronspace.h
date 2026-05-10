#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <numbers>
#include <utility>

#include "DASPi-rotation.h"
#include "DASPi-pointdata2di.h"
#include "DASPi-pointdata3dd.h"
#include "DASPi-directiondata.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-sphericalspace.h"

namespace DASPi {

// Compile-time constants.
inline constexpr double kPi{std::numbers::pi};
inline constexpr double phi{std::numbers::phi};

inline constexpr double cosPiOver5{0.80901699437494742410};
inline constexpr double sinPiOver5{0.58778525229247312917};

inline constexpr double cosPiOver7{0.90096886790241912624};
inline constexpr double sinPiOver7{0.43388373911755812048};

inline constexpr double cos0Pi{1.0};
inline constexpr double sin0Pi{0.0};

inline constexpr double cos1Pi{0.0};
inline constexpr double sin1Pi{1.0};

/*
 * Based on the facet triangle spanning across the smallest sensor dimension,
 * which is the height in this case.
 *
 * NOTE:
 * sensorHeightValue_ must be visible before this header is included,
 * or this should be moved into a space/class that owns sensorHeightValue_.
 */
inline constexpr double maskRadius{
    std::numbers::sqrt3 / 3.0 * sensorHeightValue_
};

namespace detail {

struct IcosahedronTables {
    static constexpr std::size_t verticesN_{12};
    static constexpr std::size_t facetsN_{20};
    static constexpr std::size_t edgesN_{30};

    // Number of vertices per face.
    static constexpr std::size_t verticesPerFaceN_{3};

    // Coordinate dimension.
    static constexpr std::size_t dim_{3};

    /*
     * Based on the facet triangle spanning across the smallest sensor dimension,
     * which is the height in this case.
     */
    static constexpr double overlapAdjustor_{0.75};

    static constexpr double pixelsPerNormalToNormalAngle_{
        overlapAdjustor_ * static_cast<double>(sensorHeightValue_)
    };

    // In radians.
    // 41.81031489577859647602 degrees.
    // normalToNormalAngle = acos(sqrt(5.0) / 3.0)
    static constexpr double normalToNormalAngle_{0.72972765622696635559};

    static constexpr double pixelsPerRadian_{
        pixelsPerNormalToNormalAngle_ / normalToNormalAngle_
    };

    static inline constexpr std::array<PointData3dD, verticesN_> vertices_{{
        {-1.0,  phi, 0.0}, { 1.0,  phi, 0.0},
        {-1.0, -phi, 0.0}, { 1.0, -phi, 0.0},

        {0.0, -1.0,  phi}, {0.0,  1.0,  phi},
        {0.0, -1.0, -phi}, {0.0,  1.0, -phi},

        { phi, 0.0, -1.0}, { phi, 0.0,  1.0},
        {-phi, 0.0, -1.0}, {-phi, 0.0,  1.0},
    }};

    static inline constexpr std::array<
        std::array<std::size_t, verticesPerFaceN_>,
        facetsN_
    > facets_{{
        {0, 11, 5},  {0, 5, 1},   {0, 1, 7},   {0, 7, 10},  {0, 10, 11},
        {1, 5, 9},   {5, 11, 4},  {11, 10, 2}, {10, 7, 6},  {7, 1, 8},

        {3, 9, 4},   {3, 4, 2},   {3, 2, 6},   {3, 6, 8},   {3, 8, 9},
        {4, 9, 5},   {2, 4, 11},  {6, 2, 10},  {8, 6, 7},   {9, 8, 1},
    }};

    static inline constexpr std::array<MaskOrientationData, facetsN_>
    maskOrientations_{{
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},

        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},

        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},
    }};

    static inline constexpr std::array<SensorOrientationData, facetsN_>
    sensorOrientations_{{
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},

        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},
        SensorOrientationData{cos1Pi, sin1Pi},

        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
        SensorOrientationData{cos0Pi, sin0Pi},
    }};

    static inline constexpr std::array<
        std::array<std::size_t, verticesPerFaceN_>,
        facetsN_
    > facetAdjacencies_{{
        {4, 6, 1},
        {0, 5, 2},
        {3, 9, 1},
        {4, 8, 2},
        {3, 7, 0},

        {1, 15, 19},
        {0, 16, 15},
        {4, 17, 16},
        {3, 18, 17},
        {2, 19, 18},

        {14, 15, 11},
        {10, 16, 12},
        {11, 17, 13},
        {12, 18, 14},
        {13, 19, 10},

        {10, 5, 6},
        {11, 6, 7},
        {12, 7, 8},
        {13, 8, 9},
        {14, 9, 5},
    }};

    template<std::size_t FaceIndex>
    static consteval DirectionData GetDirection()
    {
        static_assert(FaceIndex < facetsN_);
    
        constexpr std::size_t v0Index = facets_[FaceIndex][0];
        constexpr std::size_t v1Index = facets_[FaceIndex][1];
        constexpr std::size_t v2Index = facets_[FaceIndex][2];
    
        constexpr const auto& v0 = vertices_[v0Index];
        constexpr const auto& v1 = vertices_[v1Index];
        constexpr const auto& v2 = vertices_[v2Index];
    
        return DirectionData{
            (v0.x_ + v1.x_ + v2.x_) / static_cast<double>(verticesPerFaceN_),
            (v0.y_ + v1.y_ + v2.y_) / static_cast<double>(verticesPerFaceN_),
            (v0.z_ + v1.z_ + v2.z_) / static_cast<double>(verticesPerFaceN_)
        };
    }

    template<std::size_t... FaceIndices>
    static consteval std::array<DirectionData, facetsN_>
    MakeFacetDirectionsImpl(std::index_sequence<FaceIndices...>)
    {
        return {{
            GetDirection<FaceIndices>()...
        }};
    }

    static consteval std::array<DirectionData, facetsN_>
    MakeFacetDirections()
    {
        return MakeFacetDirectionsImpl(
            std::make_index_sequence<facetsN_>{}
        );
    }
};

// Keep this outside the struct so IcosahedronTables is complete
// before MakeFacetDirections() is evaluated.
inline constexpr std::array<DirectionData, IcosahedronTables::facetsN_>
facetDirections_{
    IcosahedronTables::MakeFacetDirections()
};

} // namespace detail

template<unsigned int FacetIndex>
class IcosahedronSpace : public SphericalSpace<FacetIndex>
{
public:
    using SubSpace_t = OverlapSpace< detail::IcosahedronTables::verticesPerFaceN_, detail::IcosahedronTables::maskOrientations_[FacetIndex], detail::IcosahedronTables::sensorOrientations_[FacetIndex]>;
    
    static_assert(FacetIndex < detail::IcosahedronTables::facetsN_);

    static constexpr unsigned int facetIndex_{FacetIndex};

    static constexpr std::size_t verticesN_{
        detail::IcosahedronTables::verticesN_
    };

    static constexpr std::size_t facetsN_{
        detail::IcosahedronTables::facetsN_
    };

    static constexpr std::size_t edgesN_{
        detail::IcosahedronTables::edgesN_
    };
    
    static constexpr std::size_t verticesPerFaceN_{
        detail::IcosahedronTables::verticesPerFaceN_
    };

    static constexpr std::size_t dim_{
        detail::IcosahedronTables::dim_
    };

    static constexpr DirectionData direction_{
        detail::facetDirections_[FacetIndex]
    };

    static constexpr MaskOrientationData maskOrientation_{
        detail::IcosahedronTables::maskOrientations_[FacetIndex]
    };

    static constexpr SensorOrientationData sensorOrientation_{
        detail::IcosahedronTables::sensorOrientations_[FacetIndex]
    };

    static constexpr auto adjacentFacets_{
        detail::IcosahedronTables::facetAdjacencies_[FacetIndex]
    };

    static constexpr Matrix3dData ImageTransformMatrix{
        MatrixMultiply(
            RotationYFromSinCos(cosPiOver5, sinPiOver5),
            RotationXFromSinCos(cosPiOver7, sinPiOver7)
        )
    };
};

template<class Space>
concept IcosahedronSpace_t =
    requires {
        { Space::facetIndex_ } -> std::convertible_to<unsigned int>;
    };

template<class Space>
using MakeIcosahedronSpace =
    IcosahedronSpace<Space::facetIndex_>;

} // namespace DASPi

#include "DASPi-icosahedronspace.tpp"
