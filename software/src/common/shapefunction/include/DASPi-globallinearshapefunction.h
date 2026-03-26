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
#include "DASPi-localcoordinates.h"

//#define VERBATIUM_COUT

namespace DASPi {
    
    //struct Point : public PointData {
        //using PointData::PointData; // inherit constructor
        //using PointData::operator==;
    //};
    //struct Direction : public DirectionData {
        //using DirectionData::DirectionData; // inherit constructor
        //using DirectionData::operator==;
    //};
    //struct Index : public IndexData {
        //using IndexData::IndexData; // inherit constructor
        //using IndexData::operator==;
    //};
    
    static constexpr size_t sensorWidthValue_ = 1456;//1536;
    static constexpr size_t sensorHeightValue_ = 1088;//864;
    
    template<PointData center, DirectionData direction>
    class GlobalLinearShapeFunction
    {
    public:
        //Local Coordinates structures
        struct Point : public PointData {
            using PointData::PointData; // inherit constructor
            using PointData::operator==;
            constexpr Point(const PointData& pd) : PointData(pd) {};
        };
        struct Direction : public DirectionData {
            using DirectionData::DirectionData; // inherit constructor
            using DirectionData::operator==;
            constexpr Direction(const DirectionData& dd) : DirectionData(dd) {};
        };
        struct Index : public IndexData {
            using IndexData::IndexData; // inherit constructor
            using IndexData::operator==;
            
            //// Add conversion from EquilateralTriangularShapeFunction::Index
            //Index(const EquilateralTriangularShapeFunction::Index& other)
                //: IndexData(other.value()) {}
        };
    
        //static constexpr size_t sensorWidthValue_ = 1536;
        //static constexpr size_t sensorHeightValue_ = 864;
    
        static constexpr Index sensorWidth() { return Index{sensorWidthValue_}; }
        static constexpr Index sensorHeight() { return Index{sensorHeightValue_}; }
    
        template<typename localIndex_t>
        using IndexMap = std::array<std::array<localIndex_t, sensorWidthValue_>, sensorHeightValue_>;
        template<typename globalIndex_t>
        using IndexLinearMax = std::array<globalIndex_t, sensorWidthValue_ * sensorHeightValue_>;
        
        using GlobalLinearShapeFunction_t = GlobalLinearShapeFunction<center, direction>;
        using ShapeDefiningPoints = std::array<GlobalLinearShapeFunction_t::Point, 4>;
    
        static constexpr Point center_{center};
        static constexpr Direction direction_{direction};
        ShapeDefiningPoints shapeDefiningPoints_;
        
        std::unique_ptr<IndexMap<Index>> indexMap_; // Index in this case is a local Index.
        std::unique_ptr<IndexLinearMax<Index>> indexLinearMax_;
    
        static ShapeDefiningPoints 
        DefineShapeDefiningPoints();
        
        static constexpr uint16_t
        Mask(const Point &p);
    
        template<typename localIndex_t>
        static constexpr std::unique_ptr<IndexMap<localIndex_t>> 
        GenerateIndexMap(
	        const std::function<uint16_t(const Point&)> &maskFunc
        );     
        
        template<typename localIndex_t>
        static constexpr std::unique_ptr<IndexLinearMax<Index/*index_t*/>> 
        GenerateIndexLinear(const IndexMap<localIndex_t> *indexMap);

        static constexpr /*GlobalLinearShapeFunction::*/Point Transform(/*GlobalLinearShapeFunction::*/Index index);
    
        static constexpr Index Transform(const Point &p);
    
        GlobalLinearShapeFunction();
    
        size_t size();
    
        template<typename T0>
        std::vector<uint16_t> FrameBufferMask(const T0 &frameBuffer, const IndexLinearMax<GlobalLinearShapeFunction_t::Index> *indexLinearMax, size_t indexLinearSize);
    
        static void FrameBufferMaskChunked(std::span<uint16_t> input,
                                         IndexLinearMax<GlobalLinearShapeFunction_t::Index>::iterator indexBegin,
                                         IndexLinearMax<GlobalLinearShapeFunction_t::Index>::iterator indexEnd,
                                         uint16_t* outputBuffer);
      
        template<typename T0>
        static std::vector<uint16_t> FrameBufferUnmask(const T0 &frameBuffer, const IndexLinearMax<GlobalLinearShapeFunction_t::Index> *indexLinearMax);
        
        template<typename index_t>  
        static void SaveArrayToFile(const IndexLinearMax<index_t> *data, const std::string& filename);
       
        template<typename index_t>
        static void SaveArrayToFile(const IndexMap<index_t> *data, const std::string& filename);
    };
    
    inline GlobalLinearShapeFunction<
        PointData{
            static_cast<size_t>(0.5*sensorWidthValue_),
            static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)
        },
        DirectionData { 
            static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)),
            static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))
        }
    > globalLinearShapeFunctionInstance0;
    
    inline GlobalLinearShapeFunction<
        PointData{
            static_cast<size_t>(0.5*sensorWidthValue_),
            static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)
        },
        DirectionData { 
            -1 * static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)),
            -1 * static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))
         }
    > globalLinearShapeFunctionInstance1;
    
}; // namespace DASPi
#include "DASPi-globallinearshapefunction.tpp"
