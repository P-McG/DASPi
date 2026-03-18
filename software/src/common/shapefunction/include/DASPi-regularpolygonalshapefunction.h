#pragma once
#include <numbers>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ranges>
#include <array>
#include <tuple>
#include <cstddef>
#include <numbers>
#include <fstream>
        
#include "DASPi-boundingbox.h"
#include "DASPi-localcoordinates.h"
#include "DASPi-globallinearshapefunction.h"

#define VERBATIUM_COUT

namespace DASPi{
	
	// RegularPolygonalShapeFunction
	/*
	 * Provides:
	 * - global to local transformations
	 * - masking to local frame
	 * -  
	 * 
	 * Assumes:
	 * - the cameras are position in proper rotation due to the
	 * low speed of arbitrary rotational transforms.
	 */
	 template<size_t n, PointData center, DirectionData direction>
	class RegularPolygonalShapeFunction : virtual public GlobalLinearShapeFunction<center, direction>{

       //static constexpr size_t n{6};
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
            
			// Accept Global index as input
            Index( const typename GlobalLinearShapeFunction<center, direction>::Index& other)
                : IndexData(other.value()) {}
        };

        using GlobalLinearShapeFunction_t = GlobalLinearShapeFunction<center, direction>;
        using Point = RegularPolygonalShapeFunction<n, center, direction>::Point;
        using Direction = RegularPolygonalShapeFunction<n, center, direction>::Direction;
        //using Index = RegularPolygonalShapeFunction<n, center, direction>::Index;
        //using Index = GlobalLinearShapeFunction<center, direction>::Index;
        using IndexMap = typename GlobalLinearShapeFunction_t::IndexMap<Index>;//index is a local index
        using IndexLinearMax = typename GlobalLinearShapeFunction_t::IndexLinearMax<typename GlobalLinearShapeFunction_t::Index>;//index is a global index
        using ShapeDefiningPoints = std::array<typename GlobalLinearShapeFunction_t::Point, n>;
        
		// Common member variables
		GlobalLinearShapeFunction_t::Point center_{typename GlobalLinearShapeFunction_t::Point(center)};
		GlobalLinearShapeFunction_t::Direction direction_{typename GlobalLinearShapeFunction_t::Direction(direction)};
        ShapeDefiningPoints shapeDefiningPoints_;
        std::unique_ptr<IndexMap> indexMap_;
        std::unique_ptr<IndexLinearMax> indexLinearMax_;
       
    public:
       RegularPolygonalShapeFunction(
            //GlobalLinearShapeFunction_t::Point center = GlobalLinearShapeFunction_t::Point{static_cast<size_t>(0.5*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
		    //GlobalLinearShapeFunction_t::Direction direction = GlobalLinearShapeFunction_t::Direction { static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)), static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))}
        );
        
    protected:
		void DefineShapeDefiningPoints(/*const GlobalLinearShapeFunction_t::Direction &direction, const GlobalLinearShapeFunction_t::Point &center*/);
	    void GenerateIndexMap();
        void GenerateIndexLinear();
		constexpr uint16_t Mask(const GlobalLinearShapeFunction_t::Point &p);
        static constexpr double TriangleArea(const GlobalLinearShapeFunction_t::Point &p1, const GlobalLinearShapeFunction_t::Point &p2, const GlobalLinearShapeFunction_t::Point &p3);
        constexpr std::array<double, n> ComputeFacetAreas(const GlobalLinearShapeFunction_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints);
        constexpr bool IsInside(const GlobalLinearShapeFunction_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints);
		
	public:		        
		size_t size() const;
        template <typename T0>
            auto FrameBufferUnmask(T0 &&frameBuffer);
        template<typename frameBuffer_t>
            std::vector<uint16_t> FrameBufferMask(frameBuffer_t &&frameBuffer) ;
      void FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        );
    };
    
    inline RegularPolygonalShapeFunction<
        3,
        PointData{static_cast<size_t>((+1.0/2.0)*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
        DirectionData { static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(0.0*std::numbers::pi/3.0)), static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(0.0*std::numbers::pi/3.0))}
    > regularPolygonalShapeFunctionInstance0;
    
    inline RegularPolygonalShapeFunction<
        3,
        PointData{static_cast<size_t>(0.5*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
        DirectionData { static_cast<long>(0.75*(1.0/2.0)*sensorHeightValue_*sin(0.0*std::numbers::pi/3.0)), static_cast<long>(0.75*(1.0/2.0)*sensorHeightValue_*cos(0.0*std::numbers::pi/3.0))}
    > regularPolygonalShapeFunctionInstance1;
    
};//ending namespace DASPi
#include "DASPi-regularpolygonalshapefunction.tpp"
