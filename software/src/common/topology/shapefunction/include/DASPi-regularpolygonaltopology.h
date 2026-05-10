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
#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-indexdata.h"
#include "DASPi-globallinearspace.h"
#include "DASPi-globallineartopology.h"
#include "DASPi-regularpolygonalspace.h"

#define VERBATIUM_COUT

namespace DASPi{
	
	// RegularPolygonalTopology
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
	 template<RegularPolygonalSpace_t Space>
	class RegularPolygonalTopology : virtual public GlobalLinearTopology<MakeGlobalLinearSpace<Space>>{

       //static constexpr size_t n{6};
    public:
	   using Space_t = Space;

      //Local Coordinates structures
        struct Point : public PointData2dI {
            using PointData2dI::PointData2dI; // inherit constructor
            using PointData2dI::operator==;
            constexpr Point(const PointData2dI& pd) : PointData2dI(pd) {};
        };
        struct SensorOrientation : public SensorOrientationData {
            using SensorOrientationData::SensorOrientationData; // inherit constructor
            using SensorOrientationData::operator==;
            constexpr SensorOrientation(const SensorOrientationData& dd) : SensorOrientationData(dd) {};
        };
        struct Index : public IndexData {
            using IndexData::IndexData; // inherit constructor
            using IndexData::operator==;
            
			// Accept Global index as input
            Index( const typename GlobalLinearTopology<Space>::Index& other)
                : IndexData(other.value()) {}
        };

        using GlobalLinearTopology_t = GlobalLinearTopology<MakeGlobalLinearSpace<Space>>;
        using Point = RegularPolygonalTopology<Space>::Point;
        using SensorOrientation = RegularPolygonalTopology<Space>::SensorOrientation;
        //using Index = RegularPolygonalTopology<n, center, orientation>::Index;
        //using Index = GlobalLinearTopology<center, orientation>::Index;
        using IndexMap = typename GlobalLinearTopology_t::IndexMap<Index>;//index is a local index
        using IndexLinearMax = typename GlobalLinearTopology_t::IndexLinearMax<typename GlobalLinearTopology_t::Index>;//index is a global index
        using ShapeDefiningPoints = std::array<typename GlobalLinearTopology_t::Point, Space::n_>;
        
		// Common member variables
		GlobalLinearTopology_t::Point center_{typename GlobalLinearTopology_t::Point(Space::center_)};
		GlobalLinearTopology_t::Orientation orientation_{typename GlobalLinearTopology_t::Orientation(Space::orientation_)};
        ShapeDefiningPoints shapeDefiningPoints_;
        std::unique_ptr<IndexMap> indexMap_;
        std::unique_ptr<IndexLinearMax> indexLinearMax_;
       
    public:
       RegularPolygonalTopology(
            //GlobalLinearTopology_t::Point center = GlobalLinearTopology_t::Point{static_cast<size_t>(0.5*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
		    //GlobalLinearTopology_t::Orientation orientation = GlobalLinearTopology_t::Orientation { static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)), static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))}
        );
        
    protected:
		void DefineShapeDefiningPoints(/*const GlobalLinearTopology_t::Orientation &orientation, const GlobalLinearTopology_t::Point &center*/);
	    void GenerateIndexMap();
        void GenerateIndexLinear();
		constexpr uint16_t Mask(const GlobalLinearTopology_t::Point &p);
        static constexpr double TriangleArea(const GlobalLinearTopology_t::Point &p1, const GlobalLinearTopology_t::Point &p2, const GlobalLinearTopology_t::Point &p3);
        constexpr std::array<double, Space::n_> ComputeFacetAreas(const GlobalLinearTopology_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints);
        constexpr bool IsInside(const GlobalLinearTopology_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints);
		
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
    
    //inline RegularPolygonalTopology<
        //3,
        //PointData2dI{static_cast<size_t>((+1.0/2.0)*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
        //OrientationData { static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(0.0*std::numbers::pi/3.0)), static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(0.0*std::numbers::pi/3.0))}
    //> regularPolygonalTopologyInstance0;
    
    //inline RegularPolygonalTopology<
        //3,
        //PointData2dI{static_cast<size_t>(0.5*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
        //OrientationData { static_cast<long>(0.75*(1.0/2.0)*sensorHeightValue_*sin(0.0*std::numbers::pi/3.0)), static_cast<long>(0.75*(1.0/2.0)*sensorHeightValue_*cos(0.0*std::numbers::pi/3.0))}
    //> regularPolygonalTopologyInstance1;
    
};//ending namespace DASPi
#include "DASPi-regularpolygonaltopology.tpp"
