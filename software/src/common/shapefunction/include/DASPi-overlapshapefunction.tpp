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
		OverlapShapeFunction<n, center, direction, nonOverlapScale>::OverlapShapeFunction()
       //:Facet_t(
            //GlobalLinearShapeFunction_t::Point{
				//static_cast<size_t>(0.5*GlobalLinearShapeFunction_t::sensorWidthValue_), 
				//static_cast<size_t>(0.5*GlobalLinearShapeFunction_t::sensorHeightValue_)
			//},
		    //GlobalLinearShapeFunction_t::Direction { 
				//static_cast<long>((1.0/2.0)*GlobalLinearShapeFunction_t::sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / n)), 
				//static_cast<long>((1.0/2.0)*GlobalLinearShapeFunction_t::sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / n))
			//}
        //),
        //NonOverlapFacet_t(
            //GlobalLinearShapeFunction_t::Point{
				//static_cast<size_t>(0.5*GlobalLinearShapeFunction_t::sensorWidthValue_), 
				//static_cast<size_t>(0.5*GlobalLinearShapeFunction_t::sensorHeightValue_)
			//},
		    //GlobalLinearShapeFunction_t::Direction { 
				//static_cast<long>((1.0/2.0)*nonOverlapScale*GlobalLinearShapeFunction_t::sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / n)), 
				//static_cast<long>((1.0/2.0)*nonOverlapScale*GlobalLinearShapeFunction_t::sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / n))
			//}
        //)
       {
            log_verbose("[OverlapShapeFunction::OverlapShapeFunction]");
            
            //indexMaps_.reserve(n);
			for (size_t i = 0; i < n; ++i) {
			    indexMaps_[i] = std::make_unique<IndexMap>();
			}
			
			//indexLinearMaxs_.reserve(n);
			for (size_t i = 0; i < n; ++i) {
			    indexLinearMaxs_[i] = std::make_unique<IndexLinearMax>();
			}
			
            log_verbose("Call GenerateIndexMap");
            GenerateIndexMap();
            //for(size_t i = 0; i < n; i++) 
	            //GlobalLinearShapeFunction::SaveArrayToFile(indexMaps_[i].get(), "indexMaps" + std::to_string(i) + ".txt");
			log_verbose("Call GenerateIndexLinear");
            GenerateIndexLinear();
            //for(size_t i = 0; i < n; i++) 
	            //GlobalLinearShapeFunction::SaveArrayToFile(indexLinearMaxs_[i].get(), "indexLinearMaxs" + std::to_string(i) + ".txt");
        };
        	
       	// GenerateIndexMap
		/*
		 */
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
	    void OverlapShapeFunction<n, center, direction, nonOverlapScale>::GenerateIndexMap(){
            
            using G = GlobalLinearShapeFunction<center, direction>;
            //using Index = typename G::Index;
            using Point = typename G::Point;
            
			log_verbose("[OverlapShapeFunction::GenerateIndexMap]");
			
            for(size_t overlapRegion = 0; overlapRegion < n; overlapRegion++){
				const size_t region = overlapRegion;
				indexMaps_[overlapRegion] = GlobalLinearShapeFunction<center, direction>::template GenerateIndexMap<Index>(
					[this, region](const /*typename GlobalLinearShapeFunction<center, direction>::*/Point &p){
						return Mask(p, region);
				});
			}
		};
		 
        // GenerateIndexLinear
        /*
         */
         template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
         void OverlapShapeFunction<n, center, direction, nonOverlapScale>::GenerateIndexLinear(){
             
            log_verbose("[OverlapShapeFunction::GenerateIndexLinear]");
			for(size_t overlapRegion = 0; overlapRegion < n; overlapRegion++){
	            indexLinearMaxs_[overlapRegion] = /*typename*/ GlobalLinearShapeFunction_t::GenerateIndexLinear/*<Index>*/(indexMaps_[overlapRegion].get());
			}
		};
        
        //Dot
		template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
		double OverlapShapeFunction<n, center, direction, nonOverlapScale>::Dot(const std::vector<double>& a, const std::vector<double>& b){
		    double result = 0.0;
		    for (size_t i = 0; i < a.size(); ++i)
		        result += a[i] * b[i];
		    return result;
		}
		
        //Norm
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
		double OverlapShapeFunction<n, center, direction, nonOverlapScale>::Norm(const std::vector<double>& v){
		    return std::sqrt(Dot(v, v));
		}
		
        //AngleBetween
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
		double OverlapShapeFunction<n, center, direction, nonOverlapScale>::AngleBetween(
            const std::vector<double>& a, 
            const std::vector<double>& b
        ){
		    double cosTheta = Dot(a, b) / (Norm(a) * Norm(b));
		#if __cplusplus >= 201703L
		    cosTheta = std::clamp(cosTheta, -1.0, 1.0);  // C++17
		#else
		    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));  // For older versions
		#endif
		    return std::acos(cosTheta);  // In radians
		}
		
        // SignedAngle2D
		/* 
         * Computes signed angle from a → b, CCW
         */
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
		double OverlapShapeFunction<n, center, direction, nonOverlapScale>::SignedAngle2D(const std::vector<double>& from, const std::vector<double>& to){
		    return std::atan2(to[0]*from[1] - to[1]*from[0], Dot(from, to));
		}
		
		//// Mask
		///*
		 //*/
         //template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
 		//uint16_t OverlapShapeFunction<n, center, direction, nonOverlapScale>::Mask(
	 		//const size_t overlapRegion
 		//){
	 		 //return Mask(p, overlapRegion);
		//}

		// Mask
		/*
		 */
         template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
 	    uint16_t OverlapShapeFunction<n, center, direction, nonOverlapScale>::Mask(
            const GlobalLinearShapeFunction_t::Point &p,
	 		const size_t overlapRegion
 		){
			//log_verbose("[OverlapShapeFunction::Mask] overlapRegion:" + std::to_string(overlapRegion));//EXCESSIVE LOGGING

			std::vector<double> vector0{ 
					std::vector<double>{
						static_cast<double>(direction.deltaX()),
						static_cast<double>(direction.deltaY())
			}};

			std::vector<double> vector1{
					 std::vector<double>{
							 static_cast<double>(p.x()) - static_cast<double>(center.x()), 
							 static_cast<double>(p.y()) - static_cast<double>(center.y())
			}};

		    double theta{SignedAngle2D(vector0, vector1)};
			theta = std::fmod(theta + 2.0 * std::numbers::pi, 2.0 * std::numbers::pi);
			    
			double lowerLimit{2.0 * std::numbers::pi * double(overlapRegion)  / double(n)};
			double upperLimit{2.0 * std::numbers::pi * double(overlapRegion + 1) / double(n)};

			uint16_t overlapRegionMask {
				((lowerLimit <= theta) && (theta < upperLimit))
			    ? uint16_t{0XFF} : uint16_t{0x00}
			};
			return overlapRegionMask
				& facet_t::Mask(p) 
				& static_cast<uint16_t>(~nonOverlapFacet_t::Mask(p));
		};

        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
		size_t OverlapShapeFunction<n, center, direction, nonOverlapScale>::size(const size_t overlapRegion) const {
			return (*indexMaps_[overlapRegion])[GlobalLinearShapeFunction_t::sensorHeight().value()-1UL][GlobalLinearShapeFunction_t::sensorWidth().value()-1UL].value()+1UL;
		};

        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
        template <typename frameBuffer_t>
        auto OverlapShapeFunction<n, center, direction, nonOverlapScale>::FrameBufferUnmask(
            frameBuffer_t &&frameBuffer, size_t overlapRegion
        ){
			 //log_verbose("[OverlapShapeFunction::FrameBufferUnmask]");
			 
			 //check to see if overlapRegion is within limits
			 if(overlapRegion >= n_){
                 std::cerr << "[ERROR] overlapRegion out of bounds" << std::endl;
                 std::terminate(); // or throw
			 }
			 
             return static_cast<facet_t::GlobalLinearShapeFunction_t*>(this)->FrameBufferUnmask( 
                frameBuffer,
                indexLinearMaxs_[overlapRegion].get()
             );
        };

        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
        template<typename frameBuffer_t>
        std::vector<uint16_t> OverlapShapeFunction<n, center, direction, nonOverlapScale>::FrameBufferMask(frameBuffer_t &&frameBuffer, size_t overlapRegion) {
            //log_verbose("[OverlapShapeFunction::FrameBufferMask2]");
			std::vector<uint16_t>output(frameBuffer.size());

            {
				size_t numThreads{4};
                std::vector<std::thread> workers;
        
                const size_t total = size(overlapRegion);// total number of output pixels

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
                            
                            FrameBufferMaskChunked(frameBuffer, overlapRegion, start, end, output.data() + start);
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
                    
            output.resize(size(overlapRegion));
            return output; 
       }
        
      // FrameBufferMaskChunked
      /*
       */
       template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
      void OverlapShapeFunction<n, center, direction, nonOverlapScale>::FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t overlapRegion,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        ) {
            //log_verbose("[OverlapShapeFunction::FrameBufferMaskChunked]");
            size_t maxSize = size(overlapRegion);//testing
        
            if (start >= maxSize) {
                std::cerr << "[Chunk] start >= size()! start=" << start << " size=" << maxSize << std::endl;
                exit(1);
            }
        
            size_t clampedEnd = std::min(end, maxSize);
        //#ifdef VERBATIUM_COUT
            //std::cout << "[Chunk] start=" << start << ", clampedEnd=" << clampedEnd << std::endl;
        //#endif
        
            typename IndexLinearMax::iterator chunkBegin = indexLinearMaxs_[overlapRegion]->begin() + start;
            typename IndexLinearMax::iterator chunkEnd = indexLinearMaxs_[overlapRegion]->begin() + clampedEnd;
        
            static_cast<GlobalLinearShapeFunction_t*>(this)->FrameBufferMaskChunked(
                input,
                chunkBegin,
                chunkEnd,
                outputBuffer
            );
        }
		
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
        template<typename frameBuffer_t, typename DstT>
        void OverlapShapeFunction<n, center, direction, nonOverlapScale>::FrameBufferScatterTo(
            frameBuffer_t&& frameBuffer,
            size_t overlapRegion,
            DstT* dst,
            size_t dstSize)
        {
            if (overlapRegion >= n) {
                throw std::runtime_error("FrameBufferScatterTo: invalid overlap region.");
            }
        
            using base_t = GlobalLinearShapeFunction<center, direction>;
        
            base_t::FrameBufferScatterTo(
                std::forward<frameBuffer_t>(frameBuffer),
                indexLinearMaxs_[overlapRegion].get(),
                dst,
                dstSize);
        }
                
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
        template<typename frameBuffer_t, typename DstT>
        void OverlapShapeFunction<n, center, direction, nonOverlapScale>::FrameBufferScatterAccumulateTo(
            frameBuffer_t&& frameBuffer,
            size_t overlapRegion,
            DstT* dst,
            size_t dstSize)
        {
            if (overlapRegion >= n) {
                throw std::runtime_error("FrameBufferScatterAccumulateTo: invalid overlap region.");
            }
        
            using base_t = GlobalLinearShapeFunction<center, direction>;
        
            base_t::FrameBufferScatterAccumulateTo(
                std::forward<frameBuffer_t>(frameBuffer),
                indexLinearMaxs_[overlapRegion].get(),
                dst,
                dstSize);
        }
           
        template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
        void OverlapShapeFunction<n, center, direction, nonOverlapScale>::InitializeMasks(){
            for (size_t i = 0; i < n + 1; ++i) {
                maskNonOverlap_[i] = cv::Mat(sensorHeightValue_, sensorWidthValue_, CV_8U, cv::Scalar(0));
                maskOverlap_[i]    = cv::Mat(sensorHeightValue_, sensorWidthValue_, CV_8U, cv::Scalar(0));
            }
        
            // Fill exact valid pixels from sf_ / sfdp_ here.
            
        }
};//ending namespace DASPi
#include "DASPi-regularpolygonalshapefunction.tpp"
