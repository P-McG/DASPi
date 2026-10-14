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

inline constexpr double cos2PiOver3{0.5};
inline constexpr double sin2PiOver3{0.86602540378};


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
    
    // In radians.
    // 41.81031489577859647602 degrees.
    // normalToNormalAngle = acos(sqrt(5.0) / 3.0)
    static constexpr double normalToNormalAngle_{
        0.72972765622696635559
    };
    
    static constexpr double geometricPixelsPerNormalToNormalAngle_{
        static_cast<double>(sensorHeightValue_) *
        std::numbers::sqrt3 / 3.0
    };
    
    static constexpr double lensFovScale_{1.2 /*1.095*/}; // unitless only
    
    static constexpr double pixelsPerNormalToNormalAngle_{
        geometricPixelsPerNormalToNormalAngle_ / lensFovScale_
    };
    
    static constexpr double cameraFocalPx_{
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
        {0, 11, 5},//facet0
        {0, 5, 1},//facet1
        {1, 7, 0},//facet2
        {0, 7, 10},//facet3
        {0, 10, 11},//facet4
        {1, 5, 9},//facet5
        {5, 11, 4},//facet6
        {11, 10, 2},//facet7
        {10, 7, 6},//facet8
        {1, 8, 7},//facet9
        {3, 9, 4},//facet10
        {3, 4, 2},//facet11
        {3, 2, 6},//facet12
        {3, 6, 8},//facet13
        {3, 8, 9},//facet14
        {4, 9, 5},//facet15
        {2, 4, 11},//facet16
        {6, 2, 10},//facet17
        {8, 6, 7},//facet18
        {1, 9, 8},//facet19
    }};

    static inline constexpr std::array<MaskOrientationData, facetsN_>
    maskOrientations_{{
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face0
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face1
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face2
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face3
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face4

        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face5
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face6
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face7
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face8
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face9
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face10
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face11
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face12
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face13
        MaskOrientationData{maskRadius, cos1Pi, sin1Pi},//face14

        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face15
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face16
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face17
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face18
        MaskOrientationData{maskRadius, cos0Pi, sin0Pi},//face19
    }};

    static inline constexpr std::array<SensorOrientationData, facetsN_>
    sensorOrientations_{{
        SensorOrientationData{cos0Pi, sin0Pi},//face0
        SensorOrientationData{cos0Pi, sin0Pi},//face1
        SensorOrientationData{cos0Pi, sin0Pi},//face2
        SensorOrientationData{cos0Pi, sin0Pi},//face3
        SensorOrientationData{cos0Pi, sin0Pi},//face4

        SensorOrientationData{cos1Pi, sin1Pi},//face5
        SensorOrientationData{cos1Pi, sin1Pi},//face6
        SensorOrientationData{cos1Pi, sin1Pi},//face7
        SensorOrientationData{cos1Pi, sin1Pi},//face8
        SensorOrientationData{cos0Pi, sin0Pi},//face9
        SensorOrientationData{cos1Pi, sin1Pi},//face10
        SensorOrientationData{cos1Pi, sin1Pi},//face11
        SensorOrientationData{cos1Pi, sin1Pi},//face12
        SensorOrientationData{cos1Pi, sin1Pi},//face13
        SensorOrientationData{cos1Pi, sin1Pi},//face14

        SensorOrientationData{cos0Pi, sin0Pi},//face15
        SensorOrientationData{cos0Pi, sin0Pi},//face16
        SensorOrientationData{cos0Pi, sin0Pi},//face17
        SensorOrientationData{cos0Pi, sin0Pi},//face18
        SensorOrientationData{cos0Pi, sin0Pi},//face19
    }};

    static inline constexpr std::array<
        std::array<std::size_t, verticesPerFaceN_>,
        facetsN_
    > facetAdjacencies_{{
        {4, 6, 1},//facet0
        {0, 5, 2},//facet1
        {9, 1, 3},//facet2
        {4, 8, 2},//facet3
        {3, 7, 0},//facet4
        {1, 15, 19},//facet5
        {0, 16, 15},//facet6
        {4, 17, 16},//facet7
        {3, 18, 17},//facet8
        {19, 18, 2},//facet9
        {14, 15, 11},//facet10
        {10, 16, 12},//facet11
        {11, 17, 13},//facet12
        {12, 18, 14},//facet13
        {13, 19, 10},//facet14
        {10, 5, 6},//facet15
        {11, 6, 7},//facet16
        {12, 7, 8},//facet17
        {13, 8, 9},//facet18
        {5, 14, 9},//facet19
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
class IcosahedronSpace 
    : public SphericalSpace<FacetIndex>
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
