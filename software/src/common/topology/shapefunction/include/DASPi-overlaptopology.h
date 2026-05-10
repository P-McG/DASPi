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
#include <opencv2/opencv.hpp>
        
#include "DASPi-boundingbox.h"
#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-indexdata.h"
#include "DASPi-regularpolygonaltopology.h"
#include "DASPi-regularpolygonalspace.h"
#include "DASPi-overlapspace.h"

#define VERBATIUM_COUT

namespace DASPi{
    
    template<class Space>
    using coverageSpace = RegularPolygonalSpace<Space::n_, Space::maskOrientation_, Space::sensorOrientation_>;
    
        
    template<class Space>
    using NonOverlapSpace = RegularPolygonalSpace<
        Space::n_, 
        MaskOrientationData{ 
            Space::nonOverlapScale_ * Space::maskOrientation_.radius_, 
            Space::maskOrientation_.cosValue_,
            Space::maskOrientation_.sinValue_
        },
        Space::sensorOrientation_
    >;
    
	// OverlapTopology
	/*
	 */
	template<OverlapSpace_t Space>
	class OverlapTopology: //overlap
		public RegularPolygonalTopology<coverageSpace<Space>>,//Facet
		public RegularPolygonalTopology<NonOverlapSpace<Space>>//nonOverlapFacet
	{
			
    public:
        using Space_t = Space;
   
        //Local Coordinates structures
        struct Index : public IndexData {
            using IndexData::IndexData; // inherit constructor
            using IndexData::operator==;
            
            // Accept Global index as input
            Index(const GlobalLinearTopology<Space>::Index& other)
                : IndexData(other.value()) {}
                
			Index(size_t val) : IndexData(val) {}
        };
		
		//using GlobalLinearSpace_t = GlobalLinearSpace<Space::center_, Space::orientation_> ;
        using GlobalLinearTopology_t = GlobalLinearTopology<MakeGlobalLinearSpace<Space>>;
		
		using coverage_t = RegularPolygonalTopology<Space>;
		
		//using nonOverlapFacet = RegularPolygonalSpace<Space::n_, Space::center_, 
            //OrientationData{ 
                //static_cast<long>(Space::nonOverlapScale_ * Space::orientation_.deltaX_), 
                //static_cast<long>(Space::nonOverlapScale_ * Space::orientation_.deltaY_)
            //}
        //>;
        using NonOverlapFacet_t = RegularPolygonalTopology<NonOverlapSpace<Space>>;

        using Index = OverlapTopology::Index;
        using IndexMap = typename GlobalLinearTopology_t::IndexMap<Index>;//index is a local index.
        using IndexLinearMax = typename GlobalLinearTopology_t::IndexLinearMax<typename GlobalLinearTopology_t::Index>;// index is a global index.
        
		std::array<std::unique_ptr<IndexMap>, Space::n_> indexMaps_;
        std::array<std::unique_ptr<IndexLinearMax>, Space::n_> indexLinearMaxs_;
        std::array<cv::Mat, Space::n_ + 1> maskNonOverlap_;
        std::array<cv::Mat, Space::n_ + 1> maskOverlap_;
    public:
        OverlapTopology();
	    void GenerateIndexMap();
        void GenerateIndexLinear();
		static double Dot(const std::vector<double>& a, const std::vector<double>& b);
		static double Norm(const std::vector<double>& v);
		static double AngleBetween(const std::vector<double>& a, const std::vector<double>& b);
		static double SignedAngle2D(const std::vector<double>& from, const std::vector<double>& to);
 		uint16_t Mask(
	 		const GlobalLinearTopology_t::Point &p,
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
		
		template<typename frameBuffer_t, typename DstT>
		void FrameBufferScatterTo(
			frameBuffer_t&& frameBuffer,
			size_t overlapRegion,
			DstT* dst,
			size_t dstSize);
		
		template<typename frameBuffer_t, typename DstT>
		void FrameBufferScatterAccumulateTo(
			frameBuffer_t&& frameBuffer,
			size_t overlapRegion,
			DstT* dst,
			size_t dstSize);
        void InitializeMasks();
		bool CopyMaskNonOverlap(size_t index, cv::Mat& out) const;
		bool CopyMaskOverlap(size_t index, cv::Mat& out) const;
    };
};//ending namespace DASPi
#include "DASPi-overlaptopology.tpp"
