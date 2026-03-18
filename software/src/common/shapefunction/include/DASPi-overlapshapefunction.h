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
#include <limits>
#include <iomanip>
        
#include "DASPi-boundingbox.h"
#include "DASPi-localcoordinates.h"
#include "DASPi-regularpolygonalshapefunction.h"

#define VERBATIUM_COUT

namespace DASPi{
	
	// OverlapShapeFunction
	/*
	 */
	template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
	class OverlapShapeFunction: //overlap
		public RegularPolygonalShapeFunction<n, center, direction>,//Facet
		public RegularPolygonalShapeFunction<n, center, 
            DirectionData{ 
                static_cast<long>(nonOverlapScale * direction.deltaX_), 
                static_cast<long>(nonOverlapScale * direction.deltaY_)
        }>//nonOverlapFacet
	{
			
    public:
   
        //Local Coordinates structures
        struct Index : public IndexData {
            using IndexData::IndexData; // inherit constructor
            using IndexData::operator==;
            
            // Accept Global index as input
            Index(const GlobalLinearShapeFunction<center, direction>::Index& other)
                : IndexData(other.value()) {}
                
			Index(size_t val) : IndexData(val) {}
        };
		

        using GlobalLinearShapeFunction_t = GlobalLinearShapeFunction<center, direction>;
		using facet_t = RegularPolygonalShapeFunction<n, center, direction>;
        using nonOverlapFacet_t = RegularPolygonalShapeFunction<n, center, 
            DirectionData{ 
                static_cast<long>(nonOverlapScale * direction.deltaX_), 
                static_cast<long>(nonOverlapScale * direction.deltaY_)
            }
        >;
        //using Overlap = RegularPolygonalShapeFunction<n>;

        using Index = OverlapShapeFunction::Index;
        using IndexMap = typename GlobalLinearShapeFunction_t::IndexMap<Index>;//index is a local index.
        using IndexLinearMax = typename GlobalLinearShapeFunction_t::IndexLinearMax<typename GlobalLinearShapeFunction_t::Index>;// index is a global index.
        
		static constexpr size_t n_ = n;
		static constexpr double nonOverlapScale_ = nonOverlapScale;
		std::array<std::unique_ptr<IndexMap>, n> indexMaps_;
        std::array<std::unique_ptr<IndexLinearMax>, n> indexLinearMaxs_;
    public:
        OverlapShapeFunction();
	    void GenerateIndexMap();
        void GenerateIndexLinear();
		static double Dot(const std::vector<double>& a, const std::vector<double>& b);
		static double Norm(const std::vector<double>& v);
		static double AngleBetween(const std::vector<double>& a, const std::vector<double>& b);
		static double SignedAngle2D(const std::vector<double>& from, const std::vector<double>& to);
 		uint16_t Mask(
	 		const GlobalLinearShapeFunction_t::Point &p,
	 		const size_t overlapRegion
 		);
 	    //uint16_t Mask(
	 		//const size_t overlapRegion
 		//);
		size_t size(const size_t overlapRegion) const;
        template <typename frameBuffer_t>
        auto FrameBufferUnmask(frameBuffer_t &&frameBuffer, size_t overlapRegion);
        template<typename frameBuffer_t>
        std::vector<uint16_t> FrameBufferMask(frameBuffer_t &&frameBuffer, size_t overlapRegion);
        void FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t overlapRegion,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        );
    };
    
    inline OverlapShapeFunction<
        3,
        { static_cast<size_t>((+1.0/2.0)*sensorWidthValue_), 
          static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)
        },
        { static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / 3)),
          static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / 3))
        },
        0.75
    > overlapShapeFunctionInstance0;
    inline OverlapShapeFunction<
        3,
        { static_cast<size_t>((+1.0/2.0)*sensorWidthValue_), 
          static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)
        },
        { -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / 3)),
          -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / 3))
        },
        0.75
    > overlapShapeFunctionInstance1;
    
};//ending namespace DASPi
#include "DASPi-overlapshapefunction.tpp"
