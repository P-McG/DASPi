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
#include "DASPi-regularpolygonaltopology.h"

#define VERBATIUM_COUT

namespace DASPi{
	   template<RegularPolygonalSpace_t Space> 
	   RegularPolygonalTopology<Space>::RegularPolygonalTopology(
//            GlobalLinearTopology::Point center = GlobalLinearTopology::Point{static_cast<size_t>(0.5*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
//		    GlobalLinearTopology::Orientation orientation = GlobalLinearTopology::Orientation { static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)), static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))}
        )
//		:center_{center},
//        orientation_{orientation}
        {
            log_verbose("[RegularPolygonalTopology]");
            DefineShapeDefiningPoints(/*orientation_, center_*/);
            GenerateIndexMap();
            GenerateIndexLinear();
        };
          	
    	// DefineShapeDefiningPoints
    	/*
    	 * Needs to be defined in a ccw fashion.
    	 */
		template<RegularPolygonalSpace_t Space> 
		void RegularPolygonalTopology<Space>::DefineShapeDefiningPoints(
			//const GlobalLinearTopology::Orientation &orientation, const GlobalLinearTopology::Point &center
		){
            log_verbose("[RegularPolygonalTopology::ShapeDefiningPoints]");
		    double orientationMagnitude{ sqrt( pow( Space::orientation_.deltaX(), 2) + pow(Space::orientation_.deltaY(),2)) };
			double initialRotation{ atan2(Space::orientation_.deltaY(), Space::orientation_.deltaX()) };
			
			for(size_t i=0; i < Space::n_; i++){
			      shapeDefiningPoints_[i] = typename GlobalLinearTopology_t::Point{
					     static_cast<size_t>(Space::center_.x() + orientationMagnitude * cos(double(i)/double(Space::n_) * 2.0*std::numbers::pi + initialRotation)),
						 static_cast<size_t>(Space::center_.y() + orientationMagnitude * sin(double(i)/double(Space::n_) * 2.0*std::numbers::pi + initialRotation))
				  };
			}
		};

		// GenerateIndexMap
		/*
		 */
 		template<RegularPolygonalSpace_t Space> 
	    void RegularPolygonalTopology<Space>::GenerateIndexMap(){
		    using G = GlobalLinearTopology<Space>;
			//using Index = typename G::Index;
			using Point = typename G::Point;
			
			log_verbose("[RegularPolygonalTopology::GenerateIndexMap]");
			indexMap_ = G::template GenerateIndexMap<Index>([this](const Point &p){return Mask(p);});
		};
		 
        // GenerateIndexLinear
        /*
         */
 		template<RegularPolygonalSpace_t Space> 
        void 
        RegularPolygonalTopology<Space>::GenerateIndexLinear(
        ){
			
             log_verbose("[RegularPolygonalTopology::GenerateIndexLinear]");
             indexLinearMax_ = GlobalLinearTopology<MakeGlobalLinearSpace<Space>>::GenerateIndexLinear(indexMap_.get());
		};

		//Mask
		/*
		 */
 		template<RegularPolygonalSpace_t Space> 
		constexpr uint16_t RegularPolygonalTopology<Space>::Mask(const GlobalLinearTopology_t::Point &p){
		 	return (IsInside(p, shapeDefiningPoints_)) 
			       ? uint16_t{0XFF} : uint16_t{0x00};
		}; 	
		
		// TriangleArea
		/*
		 * positive when defined ccw.
		 */
 		template<RegularPolygonalSpace_t Space> 
        constexpr double RegularPolygonalTopology<Space>::TriangleArea(const GlobalLinearTopology_t::Point &p1, const GlobalLinearTopology_t::Point &p2, const GlobalLinearTopology_t::Point &p3){
		    return 0.5 *((double)p1.x() * ((double)p2.y() - (double)p3.y()) + 
		                 (double)p2.x() * ((double)p3.y() - (double)p1.y()) + 
		                 (double)p3.x() * ((double)p1.y() - (double)p2.y()));
		};
		
		//ComputeFacetAreas
		/*
		 * Needs to be defined in a ccw fashion.
		 */
 		template<RegularPolygonalSpace_t Space> 
        constexpr std::array<double, Space::n_> RegularPolygonalTopology<Space>::ComputeFacetAreas(const GlobalLinearTopology_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints){
    
			std::array<typename GlobalLinearTopology_t::Point, 3> pts;

			std::array<double, Space::n_> facetAreas;
			for(size_t i = 0; i < Space::n_; i++){
                pts[0] = p;
                pts[1] = shapeDefiningPoints[(i + 0)];
                pts[2] = shapeDefiningPoints[(i + 1) % Space::n_];			

		        facetAreas[i] = TriangleArea(pts[0], pts[1], pts[2]);
			}
		    return facetAreas;
		};
		
		// IsInside
		/*
		 */
 		template<RegularPolygonalSpace_t Space> 
        constexpr bool RegularPolygonalTopology<Space>::IsInside(const GlobalLinearTopology_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints){
		    auto facetAreas = ComputeFacetAreas(p, shapeDefiningPoints);
		    bool flag = true;
		    for(size_t i=0; i < Space::n_; i++){
		       if(facetAreas[i] < 0.0) return false;
	 	    }
		    return flag;
		};
		
		// size
 		template<RegularPolygonalSpace_t Space> 
		size_t RegularPolygonalTopology<Space>::size() const {
			return (*indexMap_)[sensorHeightValue_-1UL][sensorWidthValue_-1UL].value()+1UL;
		};
        
        // FrameBufferUnmask
 		template<RegularPolygonalSpace_t Space> 
        template <typename T0>
        auto RegularPolygonalTopology<Space>::FrameBufferUnmask(T0 &&frameBuffer){
		     //log_verbose("[RegularPolygonalTopology::FrameBufferUnmask]");
			 
			 if (!indexLinearMax_) {
				std::cerr << "[FATAL] indexLinearMax_ is null before FrameBufferUnmask!" << std::endl;
				std::terminate();
            }
			 
             return static_cast<GlobalLinearTopology_t*>(this)->FrameBufferUnmask( 
                frameBuffer,
                indexLinearMax_.get()
             );
        };
        
        // FrameBufferMask
 		template<RegularPolygonalSpace_t Space> 
        template<typename frameBuffer_t>
        std::vector<uint16_t> RegularPolygonalTopology<Space>::FrameBufferMask(frameBuffer_t &&frameBuffer) {
            //log_verbose("[RegularPolygonalTopology::FrameBufferMask2]");
			std::vector<uint16_t>output(frameBuffer.size());

            {
				size_t numThreads{4};
                std::vector<std::thread> workers;
        
                const size_t total = RegularPolygonalTopology<Space>::size(); // total number of output pixels

                const size_t chunk = total / numThreads;
        
                for (size_t i = 0; i < numThreads; ++i) {
                    size_t start = i * chunk;
                    size_t end = (i == numThreads - 1) ? total : (i + 1) * chunk;
                
                    workers.emplace_back([&, i, start, end]() {
                        cpu_set_t cpuset;
                        CPU_ZERO(&cpuset);
                        CPU_SET(3, &cpuset);
                       // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
                        pthread_setname_np(pthread_self(), "Mask");
                
                        //std::cout << "[Thread " << i << "] started, start=" << start << ", end=" << end << std::endl;
                
                        try {
                            // Write directly into the correct segment of finalResult
                            
                            RegularPolygonalTopology<Space>::FrameBufferMaskChunked(frameBuffer, start, end, output.data() + start);
                        } catch (const std::exception& e) {
                            std::cerr << "[Thread " << i << "] exception: " << e.what() << std::endl;
                        } catch (...) {
                            std::cerr << "[Thread " << i << "] unknown exception" << std::endl;
                        }
                
                        //std::cout << "[Thread " << i << "] completed" << std::endl;
                    });
                }
                
                for (auto& t : workers) t.join();
            }
                    
            output.resize(RegularPolygonalTopology<Space>::size());
            return output; 
       }
        
      // FrameBufferMaskChunked
      /*
       */
 		template<RegularPolygonalSpace_t Space> 
      void RegularPolygonalTopology<Space>::FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        ) {
            //log_verbose("[RegularPolygonalTopology::FrameBufferMaskChunked]");
            size_t maxSize = size();
        
            if (start >= maxSize) {
                std::cerr << "[Chunk] start >= size()! start=" << start << " size=" << maxSize << std::endl;
                exit(1);
            }
        
            size_t clampedEnd = std::min(end, maxSize);
        //#ifdef VERBATIUM_COUT
            //std::cout << "[Chunk] start=" << start << ", clampedEnd=" << clampedEnd << std::endl;
        //#endif
        
            typename GlobalLinearTopology_t::IndexLinearMax<typename GlobalLinearTopology_t::Index>::iterator chunkBegin = indexLinearMax_->begin() + start;
            typename GlobalLinearTopology_t::IndexLinearMax<typename GlobalLinearTopology_t::Index>::iterator chunkEnd = indexLinearMax_->begin() + clampedEnd;
        
            // Call new in-place variant of FrameBufferMaskChunked_base
            static_cast<GlobalLinearTopology_t*>(this)->FrameBufferMaskChunked(
                input,
                chunkBegin,
                chunkEnd,
                outputBuffer
            );
        }
};//ending namespace DASPi

