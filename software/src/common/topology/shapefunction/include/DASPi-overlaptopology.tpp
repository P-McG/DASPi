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
#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-indexdata.h"
#include "DASPi-regularpolygonaltopology.h"
#include "DASPi-overlapspace.h"

#define VERBATIUM_COUT

namespace DASPi{
    
	   // OverlapTopology
       /*
       */
       	template<OverlapSpace_t Space>
		OverlapTopology<Space>::OverlapTopology()
       {
            log_verbose("[OverlapTopology::OverlapTopology]");
            
            //indexMaps_.reserve(n);
			for (size_t i = 0; i < Space::n_; ++i) {
			    indexMaps_[i] = std::make_unique<IndexMap>();
			}
			
			//indexLinearMaxs_.reserve(n);
			for (size_t i = 0; i < Space::n_; ++i) {
			    indexLinearMaxs_[i] = std::make_unique<IndexLinearMax>();
			}
			
            log_verbose("Call GenerateIndexMap");
            GenerateIndexMap();
            //for(size_t i = 0; i < n; i++) 
	            //GlobalLinearTopology::SaveArrayToFile(indexMaps_[i].get(), "indexMaps" + std::to_string(i) + ".txt");
			log_verbose("Call GenerateIndexLinear");
            GenerateIndexLinear();
            //for(size_t i = 0; i < n; i++) 
	            //GlobalLinearTopology::SaveArrayToFile(indexLinearMaxs_[i].get(), "indexLinearMaxs" + std::to_string(i) + ".txt");
        };
        	
       	// GenerateIndexMap
		/*
		 */
        template<OverlapSpace_t Space>
	    void OverlapTopology<Space>::GenerateIndexMap(){
            
            using G = GlobalLinearTopology<Space>;
            //using Index = typename G::Index;
            using Point = typename G::Point;
            
			log_verbose("[OverlapTopology::GenerateIndexMap]");
			
            for(size_t overlapRegion = 0; overlapRegion < Space::n_; overlapRegion++){
				const size_t region = overlapRegion;
				indexMaps_[overlapRegion] = GlobalLinearTopology<Space>::template GenerateIndexMap<Index>(
					[this, region](const /*typename GlobalLinearTopology<center, orientation>::*/Point &p){
						return Mask(p, region);
				});
			}
		};
		 
        // GenerateIndexLinear
        /*
         */
         template<OverlapSpace_t Space>
         void OverlapTopology<Space>::GenerateIndexLinear(){
             
            log_verbose("[OverlapTopology::GenerateIndexLinear]");
			for(size_t overlapRegion = 0; overlapRegion < Space::n_; overlapRegion++){
	            indexLinearMaxs_[overlapRegion] = /*typename*/ GlobalLinearTopology_t::GenerateIndexLinear/*<Index>*/(indexMaps_[overlapRegion].get());
			}
		};
        
        //Dot
		template<OverlapSpace_t Space>
		double OverlapTopology<Space>::Dot(const std::vector<double>& a, const std::vector<double>& b){
		    double result = 0.0;
		    for (size_t i = 0; i < a.size(); ++i)
		        result += a[i] * b[i];
		    return result;
		}
		
        //Norm
        template<OverlapSpace_t Space>
		double OverlapTopology<Space>::Norm(const std::vector<double>& v){
		    return std::sqrt(Dot(v, v));
		}
		
        //AngleBetween
        template<OverlapSpace_t Space>
		double OverlapTopology<Space>::AngleBetween(
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
        template<OverlapSpace_t Space>
		double OverlapTopology<Space>::SignedAngle2D(const std::vector<double>& from, const std::vector<double>& to){
		    return std::atan2(to[0]*from[1] - to[1]*from[0], Dot(from, to));
		}
		
		//// Mask
		///*
		 //*/
         //template<size_t n, PointData2dI center, OrientationData orientation, double nonOverlapScale>
 		//uint16_t OverlapTopology<n, center, orientation, nonOverlapScale>::Mask(
	 		//const size_t overlapRegion
 		//){
	 		 //return Mask(p, overlapRegion);
		//}

		// Mask
		/*
		 */
         template<OverlapSpace_t Space>
 	    uint16_t OverlapTopology<Space>::Mask(
            const GlobalLinearTopology_t::Point &p,
	 		const size_t overlapRegion
 		){
			//log_verbose("[OverlapTopology::Mask] overlapRegion:" + std::to_string(overlapRegion));//EXCESSIVE LOGGING

			std::vector<double> vector0{ 
					std::vector<double>{
						static_cast<double>(Space::orientation_.deltaX()),
						static_cast<double>(Space::orientation_.deltaY())
			}};

			std::vector<double> vector1{
					 std::vector<double>{
							 static_cast<double>(p.x()) - static_cast<double>(Space::center_.x()), 
							 static_cast<double>(p.y()) - static_cast<double>(Space::center_.y())
			}};

		    double theta{SignedAngle2D(vector0, vector1)};
			theta = std::fmod(theta + 2.0 * std::numbers::pi, 2.0 * std::numbers::pi);
			    
			double lowerLimit{2.0 * std::numbers::pi * double(overlapRegion)  / double(Space::n_)};
			double upperLimit{2.0 * std::numbers::pi * double(overlapRegion + 1) / double(Space::n_)};

			uint16_t overlapRegionMask {
				((lowerLimit <= theta) && (theta < upperLimit))
			    ? uint16_t{0XFF} : uint16_t{0x00}
			};
			return overlapRegionMask
				& coverage_t::Mask(p) 
				& static_cast<uint16_t>(~NonOverlapFacet_t::Mask(p));
		};

        template<OverlapSpace_t Space>
		size_t OverlapTopology<Space>::size(const size_t overlapRegion) const {
			return (*indexMaps_[overlapRegion])[GlobalLinearTopology_t::sensorHeight().value()-1UL][GlobalLinearTopology_t::sensorWidth().value()-1UL].value()+1UL;
		};

        template<OverlapSpace_t Space>
        template <typename frameBuffer_t>
        auto OverlapTopology<Space>::FrameBufferUnmask(
            frameBuffer_t &&frameBuffer, size_t overlapRegion
        ){
			 //log_verbose("[OverlapTopology::FrameBufferUnmask]");
			 
			 //check to see if overlapRegion is within limits
			 if(overlapRegion >= Space::n_){
                 std::cerr << "[ERROR] overlapRegion out of bounds" << std::endl;
                 std::terminate(); // or throw
			 }
			 
             return static_cast<coverage_t::GlobalLinearTopology_t*>(this)->FrameBufferUnmask( 
                frameBuffer,
                indexLinearMaxs_[overlapRegion].get()
             );
        };

        template<OverlapSpace_t Space>
        template<typename frameBuffer_t>
        std::vector<uint16_t> OverlapTopology<Space>::FrameBufferMask(frameBuffer_t &&frameBuffer, size_t overlapRegion) {
            //log_verbose("[OverlapTopology::FrameBufferMask2]");
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
       template<OverlapSpace_t Space>
      void OverlapTopology<Space>::FrameBufferMaskChunked(
            std::span<uint16_t> input,
            size_t overlapRegion,
            size_t start,
            size_t end,
            uint16_t* outputBuffer   // New output pointer
        ) {
            //log_verbose("[OverlapTopology::FrameBufferMaskChunked]");
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
        
            static_cast<GlobalLinearTopology_t*>(this)->FrameBufferMaskChunked(
                input,
                chunkBegin,
                chunkEnd,
                outputBuffer
            );
        }
		
        template<OverlapSpace_t Space>
        template<typename frameBuffer_t, typename DstT>
        void OverlapTopology<Space>::FrameBufferScatterTo(
            frameBuffer_t&& frameBuffer,
            size_t overlapRegion,
            DstT* dst,
            size_t dstSize)
        {
            if (overlapRegion >= Space::n_) {
                throw std::runtime_error("FrameBufferScatterTo: invalid overlap region.");
            }
        
            using base_t = GlobalLinearTopology<Space>;
        
            base_t::FrameBufferScatterTo(
                std::forward<frameBuffer_t>(frameBuffer),
                indexLinearMaxs_[overlapRegion].get(),
                dst,
                dstSize);
        }
                
        template<OverlapSpace_t Space>
        template<typename frameBuffer_t, typename DstT>
        void OverlapTopology<Space>::FrameBufferScatterAccumulateTo(
            frameBuffer_t&& frameBuffer,
            size_t overlapRegion,
            DstT* dst,
            size_t dstSize)
        {
            if (overlapRegion >= Space::n_) {
                throw std::runtime_error("FrameBufferScatterAccumulateTo: invalid overlap region.");
            }
        
            using base_t = GlobalLinearTopology<Space>;
        
            base_t::FrameBufferScatterAccumulateTo(
                std::forward<frameBuffer_t>(frameBuffer),
                indexLinearMaxs_[overlapRegion].get(),
                dst,
                dstSize);
        }
           
        template<OverlapSpace_t Space>
        void OverlapTopology<Space>::InitializeMasks(){
            for (size_t i = 0; i < Space::n_ + 1; ++i) {
                maskNonOverlap_[i] = cv::Mat(sensorHeightValue_, sensorWidthValue_, CV_8U, cv::Scalar(0));
                maskOverlap_[i]    = cv::Mat(sensorHeightValue_, sensorWidthValue_, CV_8U, cv::Scalar(0));
            }
        
            // Fill exact valid pixels from sf_ / sfdp_ here.
            
        }
};//ending namespace DASPi
