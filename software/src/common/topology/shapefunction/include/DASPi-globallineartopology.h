#pragma once
#include <functional>
#include <ranges>
#include <array>
#include <algorithm>
#include <execution>
#include <vector>
#if defined(__ARM_NEON) || defined(__aarch64__)
  #include <arm_neon.h>
#endif
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <compare>
#include <iostream>
#include <cassert>
#include <fstream>
#include <cmath>

#include "DASPi-logger.h"
#include "DASPi-boundingbox.h"
#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-indexdata.h"
#include "DASPi-globallinearspace.h"

//#define VERBATIUM_COUT

namespace DASPi {
        
template<class Space>
class GlobalLinearTopology
{
public:
    static_assert(GlobalLinearSpace_t<Space>);

    using Space_t = Space;

    struct Point : public PointData2dI {
        using PointData2dI::PointData2dI;
        using PointData2dI::operator==;

        constexpr Point(const PointData2dI& pd)
            : PointData2dI(pd)
        {}
    };

    struct SensorOrientation : public SensorOrientationData {
        using SensorOrientationData::SensorOrientationData;
        using SensorOrientationData::operator==;

        constexpr SensorOrientation(const SensorOrientationData& sd)
            : SensorOrientationData(sd)
        {}
    };

    struct Index : public IndexData {
        using IndexData::IndexData;
        using IndexData::operator==;
    };

    static constexpr Index sensorWidth()
    {
        return Index{sensorWidthValue_};
    }

    static constexpr Index sensorHeight()
    {
        return Index{sensorHeightValue_};
    }

    template<typename localIndex_t>
    using IndexMap =
        std::array<std::array<localIndex_t, sensorWidthValue_>, sensorHeightValue_>;

    template<typename globalIndex_t>
    using IndexLinearMax =
        std::array<globalIndex_t, sensorWidthValue_ * sensorHeightValue_>;

    using CanonicalSpace =
        MakeGlobalLinearSpace<Space>;

    using ShapeDefiningPoints =
        std::array<Point, 4>;

    static constexpr Point sensorCenter_{
        Space::sensorCenter_
    };

    static constexpr SensorOrientation sensorOrientation_{
        Space::sensorOrientation_
    };

    ShapeDefiningPoints shapeDefiningPoints_;

    std::unique_ptr<IndexMap<Index>> indexMap_;
    std::unique_ptr<IndexLinearMax<Index>> indexLinearMax_;

    static ShapeDefiningPoints DefineShapeDefiningPoints();

    static constexpr uint16_t Mask(const Point& p);

    template<typename localIndex_t>
    static constexpr std::unique_ptr<IndexMap<localIndex_t>>
    GenerateIndexMap(
        const std::function<uint16_t(const Point&)>& maskFunc
    );

    template<typename localIndex_t>
    static constexpr std::unique_ptr<IndexLinearMax<Index>>
    GenerateIndexLinear(const IndexMap<localIndex_t>* indexMap);

    static constexpr Point Transform(Index index);

    static constexpr Index Transform(const Point& p);

    GlobalLinearTopology();

    size_t size();

    template<typename T0>
    std::vector<uint16_t> FrameBufferMask(
        const T0& frameBuffer,
        const IndexLinearMax<Index>* indexLinearMax,
        size_t indexLinearSize
    );

    static void FrameBufferMaskChunked(
        std::span<uint16_t> input,
        typename IndexLinearMax<Index>::iterator indexBegin,
        typename IndexLinearMax<Index>::iterator indexEnd,
        uint16_t* outputBuffer
    );

    template<typename T0>
    static std::vector<uint16_t> FrameBufferUnmask(
        const T0& frameBuffer,
        const IndexLinearMax<Index>* indexLinearMax
    );

    template<typename index_t>
    static void SaveArrayToFile(
        const IndexLinearMax<index_t>* data,
        const std::string& filename
    );

    template<typename index_t>
    static void SaveArrayToFile(
        const IndexMap<index_t>* data,
        const std::string& filename
    );

    template<typename SrcT, typename DstT>
    static void FrameBufferScatterTo(
        const SrcT& frameBuffer,
        const IndexLinearMax<Index>* indexLinearMax,
        DstT* dst,
        size_t dstSize
    );

    template<typename SrcT, typename DstT>
    static void FrameBufferScatterAccumulateTo(
        const SrcT& frameBuffer,
        const IndexLinearMax<Index>* indexLinearMax,
        DstT* dst,
        size_t dstSize
    );

    template<typename SrcT>
    static void FrameBufferScatterToSpan(
        const SrcT& frameBuffer,
        const IndexLinearMax<Index>* indexLinearMax,
        std::span<uint16_t> dst
    );
};
    
    //inline GlobalLinearTopology<
        //PointData2dI{
            //static_cast<size_t>(0.5*sensorWidthValue_),
            //static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)
        //},
        //OrientationData { 
            //static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)),
            //static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))
        //}
    //> globalLinearTopologyInstance0;
    
    //inline GlobalLinearTopology<
        //PointData2dI{
            //static_cast<size_t>(0.5*sensorWidthValue_),
            //static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)
        //},
        //OrientationData { 
            //-1 * static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)),
            //-1 * static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))
         //}
    //> globalLinearTopologyInstance1;
    
}; // namespace DASPi
#include "DASPi-globallineartopology.tpp"
