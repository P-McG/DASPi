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
	class RegularPolygonalTopology
		: virtual public GlobalLinearTopology<MakeGlobalLinearSpace<Space>>
	{
	public:
		using Space_t = Space;
	
		using GlobalLinearSpace_t =
			MakeGlobalLinearSpace<Space>;
	
		using GlobalLinearTopology_t =
			GlobalLinearTopology<GlobalLinearSpace_t>;
	
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
	
			Index(const typename GlobalLinearTopology_t::Index& other)
				: IndexData(other.value())
			{}
		};
	
		using IndexMap =
			typename GlobalLinearTopology_t::template IndexMap<Index>;
	
		using IndexLinearMax =
			typename GlobalLinearTopology_t::template IndexLinearMax<
				typename GlobalLinearTopology_t::Index
			>;
	
		using ShapeDefiningPoints =
			std::array<typename GlobalLinearTopology_t::Point, Space::n_>;
	
		typename GlobalLinearTopology_t::Point sensorCenter_{
			typename GlobalLinearTopology_t::Point(
				GlobalLinearSpace_t::sensorCenter_
			)
		};
	
		typename GlobalLinearTopology_t::SensorOrientation sensorOrientation_{
			typename GlobalLinearTopology_t::SensorOrientation(
				GlobalLinearSpace_t::sensorOrientation_
			)
		};
	
		ShapeDefiningPoints shapeDefiningPoints_;
		std::unique_ptr<IndexMap> indexMap_;
		std::unique_ptr<IndexLinearMax> indexLinearMax_;
	
		RegularPolygonalTopology();

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
            auto FrameBufferUnmask(T0 &&frameBuffer) const;
        template<typename frameBuffer_t>
            std::vector<uint16_t> FrameBufferMask(frameBuffer_t &&frameBuffer);
      void FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        );
    };
};//ending namespace DASPi
#include "DASPi-regularpolygonaltopology.tpp"
