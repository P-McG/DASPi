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
#include "DASPi-regularpolygonalshapefunction.h"

#define VERBATIUM_COUT

namespace DASPi{
	   template<size_t n, PointData center, DirectionData direction> 
	   RegularPolygonalShapeFunction<n, center, direction>::RegularPolygonalShapeFunction(
//            GlobalLinearShapeFunction::Point center = GlobalLinearShapeFunction::Point{static_cast<size_t>(0.5*sensorWidthValue_), static_cast<size_t>((+1.0/2.0)*sensorHeightValue_)},
//		    GlobalLinearShapeFunction::Direction direction = GlobalLinearShapeFunction::Direction { static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*sin(2.0*std::numbers::pi/12.0)), static_cast<long>((1.0/2.0)*2.0/sqrt(3)*sensorHeightValue_*cos(2.0*std::numbers::pi/12.0))}
        )
//		:center_{center},
//        direction_{direction}
        {
            log_verbose("[RegularPolygonalShapeFunction]");
            DefineShapeDefiningPoints(/*direction_, center_*/);
            GenerateIndexMap();
            GenerateIndexLinear();
        };
          	
    	// DefineShapeDefiningPoints
    	/*
    	 * Needs to be defined in a ccw fashion.
    	 */
		template<size_t n, PointData center, DirectionData direction> 
		void RegularPolygonalShapeFunction<n, center, direction>::DefineShapeDefiningPoints(
			//const GlobalLinearShapeFunction::Direction &direction, const GlobalLinearShapeFunction::Point &center
		){
            log_verbose("[RegularPolygonalShapeFunction::ShapeDefiningPoints]");
		    double directionMagnitude{ sqrt( pow( direction.deltaX(), 2) + pow(direction.deltaY(),2)) };
			double initialRotation{ atan2(direction.deltaY(), direction.deltaX()) };
			
			for(size_t i=0; i < n; i++){
			      shapeDefiningPoints_[i] = typename GlobalLinearShapeFunction_t::Point{
					     static_cast<size_t>(center.x() + directionMagnitude * cos(double(i)/double(n) * 2.0*std::numbers::pi + initialRotation)),
						 static_cast<size_t>(center.y() + directionMagnitude * sin(double(i)/double(n) * 2.0*std::numbers::pi + initialRotation))
				  };
			}
		};

		// GenerateIndexMap
		/*
		 */
 		template<size_t n, PointData center, DirectionData direction> 
	    void RegularPolygonalShapeFunction<n, center, direction>::GenerateIndexMap(){
			
		    using G = GlobalLinearShapeFunction<center, direction>;
			//using Index = typename G::Index;
			using Point = typename G::Point;
			
			log_verbose("[RegularPolygonalShapeFunction::GenerateIndexMap]");
			indexMap_ = G::template GenerateIndexMap<Index>([this](const Point &p){return Mask(p);});
		};
		 
        // GenerateIndexLinear
        /*
         */
 		template<size_t n, PointData center, DirectionData direction> 
        void 
        RegularPolygonalShapeFunction<n, center, direction>::GenerateIndexLinear(
        ){
			
             log_verbose("[RegularPolygonalShapeFunction::GenerateIndexLinear]");
             indexLinearMax_ = 
	             GlobalLinearShapeFunction<center, direction>::/*template */GenerateIndexLinear/*<Index>*/(indexMap_.get());
		};

		//Mask
		/*
		 */
 		template<size_t n, PointData center, DirectionData direction> 
		constexpr uint16_t RegularPolygonalShapeFunction<n, center, direction>::Mask(const GlobalLinearShapeFunction_t::Point &p){
		 	return (IsInside(p, shapeDefiningPoints_)) 
			       ? uint16_t{0XFF} : uint16_t{0x00};
		}; 	
		
		// TriangleArea
		/*
		 * positive when defined ccw.
		 */
 		template<size_t n, PointData center, DirectionData direction> 
        constexpr double RegularPolygonalShapeFunction<n, center, direction>::TriangleArea(const GlobalLinearShapeFunction_t::Point &p1, const GlobalLinearShapeFunction_t::Point &p2, const GlobalLinearShapeFunction_t::Point &p3){
		    return 0.5 *((double)p1.x() * ((double)p2.y() - (double)p3.y()) + 
		                 (double)p2.x() * ((double)p3.y() - (double)p1.y()) + 
		                 (double)p3.x() * ((double)p1.y() - (double)p2.y()));
		};
		
		//ComputeFacetAreas
		/*
		 * Needs to be defined in a ccw fashion.
		 */
 		template<size_t n, PointData center, DirectionData direction> 
        constexpr std::array<double, n> RegularPolygonalShapeFunction<n, center, direction>::ComputeFacetAreas(const GlobalLinearShapeFunction_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints){
    
			std::array<typename GlobalLinearShapeFunction_t::Point, 3> pts;

			std::array<double, n> facetAreas;
			for(size_t i = 0; i < n; i++){
                pts[0] = p;
                pts[1] = shapeDefiningPoints[(i + 0)];
                pts[2] = shapeDefiningPoints[(i + 1) % n];			

		        facetAreas[i] = TriangleArea(pts[0], pts[1], pts[2]);
			}
		    return facetAreas;
		};
		
		// IsInside
		/*
		 */
 		template<size_t n, PointData center, DirectionData direction> 
        constexpr bool RegularPolygonalShapeFunction<n, center, direction>::IsInside(const GlobalLinearShapeFunction_t::Point &p, const ShapeDefiningPoints &shapeDefiningPoints){
		    auto facetAreas = ComputeFacetAreas(p, shapeDefiningPoints);
		    bool flag = true;
		    for(size_t i=0; i < n; i++){
		       if(facetAreas[i] < 0.0) return false;
	 	    }
		    return flag;
		};
		
		// size
 		template<size_t n, PointData center, DirectionData direction> 
		size_t RegularPolygonalShapeFunction<n, center, direction>::size() const {
			return (*indexMap_)[sensorHeightValue_-1UL][sensorWidthValue_-1UL].value()+1UL;
		};
        
        // FrameBufferUnmask
 		template<size_t n, PointData center, DirectionData direction> 
        template <typename T0>
        auto RegularPolygonalShapeFunction<n, center, direction>::FrameBufferUnmask(T0 &&frameBuffer){
		     //log_verbose("[RegularPolygonalShapeFunction::FrameBufferUnmask]");
			 
			 if (!indexLinearMax_) {
				std::cerr << "[FATAL] indexLinearMax_ is null before FrameBufferUnmask!" << std::endl;
				std::terminate();
            }
			 
             return static_cast<GlobalLinearShapeFunction_t*>(this)->FrameBufferUnmask( 
                frameBuffer,
                indexLinearMax_.get()
             );
        };
        
        // FrameBufferMask
 		template<size_t n, PointData center, DirectionData direction> 
        template<typename frameBuffer_t>
        std::vector<uint16_t> RegularPolygonalShapeFunction<n, center, direction>::FrameBufferMask(frameBuffer_t &&frameBuffer) {
            //log_verbose("[RegularPolygonalShapeFunction::FrameBufferMask2]");
			std::vector<uint16_t>output(frameBuffer.size());

            {
				size_t numThreads{4};
                std::vector<std::thread> workers;
        
                const size_t total = RegularPolygonalShapeFunction<n, center, direction>::size(); // total number of output pixels

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
                            
                            RegularPolygonalShapeFunction<n, center, direction>::FrameBufferMaskChunked(frameBuffer, start, end, output.data() + start);
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
                    
            output.resize(RegularPolygonalShapeFunction<n, center, direction>::size());
            return output; 
       }
        
      // FrameBufferMaskChunked
      /*
       */
 		template<size_t n, PointData center, DirectionData direction> 
      void RegularPolygonalShapeFunction<n, center, direction>::FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        ) {
            //log_verbose("[RegularPolygonalShapeFunction::FrameBufferMaskChunked]");
            size_t maxSize = size();
        
            if (start >= maxSize) {
                std::cerr << "[Chunk] start >= size()! start=" << start << " size=" << maxSize << std::endl;
                exit(1);
            }
        
            size_t clampedEnd = std::min(end, maxSize);
        //#ifdef VERBATIUM_COUT
            //std::cout << "[Chunk] start=" << start << ", clampedEnd=" << clampedEnd << std::endl;
        //#endif
        
            typename GlobalLinearShapeFunction_t::IndexLinearMax<typename GlobalLinearShapeFunction_t::Index>::iterator chunkBegin = indexLinearMax_->begin() + start;
            typename GlobalLinearShapeFunction_t::IndexLinearMax<typename GlobalLinearShapeFunction_t::Index>::iterator chunkEnd = indexLinearMax_->begin() + clampedEnd;
        
            // Call new in-place variant of FrameBufferMaskChunked_base
            static_cast<GlobalLinearShapeFunction_t*>(this)->FrameBufferMaskChunked(
                input,
                chunkBegin,
                chunkEnd,
                outputBuffer
            );
        }
};//ending namespace DASPi

