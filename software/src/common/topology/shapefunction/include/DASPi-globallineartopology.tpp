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

#include "DASPi-logger.h"
#include "DASPi-boundingbox.h"
#include "DASPi-pointdata2di.h"
#include "DASPi-maskorientationdata.h"
#include "DASPi-sensororientationdata.h"
#include "DASPi-indexdata.h"

namespace DASPi{
  
    //ShapeDefiningPoints
    template<class Space>
	typename GlobalLinearTopology<Space>::ShapeDefiningPoints 
	GlobalLinearTopology<Space>::DefineShapeDefiningPoints(){
		
	    using Point = typename GlobalLinearTopology<Space>::Point;

		log_verbose("[GlobalLinearTopology::DefineShapeDefiningPoints]");

		return { 
			Point{0, 0}, 
			Point{sensorWidthValue_, 0}, 
			Point{sensorWidthValue_, sensorHeightValue_}, 
			Point{0, sensorHeightValue_} 
		};
	}

	//Mask
	template<class Space>
	constexpr uint16_t 
	GlobalLinearTopology<Space>::Mask(
		const typename GlobalLinearTopology<Space>::Point &p
	){
		return (p.x() < sensorWidthValue_ && p.y() < sensorHeightValue_) ? 0xFF : 0x00;
	}


	// GenerateIndexMap
	/*
	 * Builds a matrix that maps GlobalLinearTopology Points to there Local Indexes(e.g. EquilateralTriangularTopology).
	 */
	 template<class Space>
	template<typename localIndex_t>
	constexpr std::unique_ptr<typename GlobalLinearTopology<Space>::template IndexMap<localIndex_t> > 
	GlobalLinearTopology<Space>::GenerateIndexMap(
		const std::function<uint16_t(const typename GlobalLinearTopology<Space>::Point&)> &maskFunc
	){
		using G = GlobalLinearTopology<Space>;
	    using Point = typename G::Point;
	    using IndexMap = typename G::template IndexMap<localIndex_t>;

		log_verbose("[GlobalLinearTopology::GenerateIndexMap]");
		
		// Scan through the GlobalLinearTopology points and any within the mask mark as one.
		// Outside the mask mark as zero.
		auto indexMap = std::make_unique<IndexMap>();
		for (size_t i = 0; i < sensorHeightValue_; i++) {
			for (size_t j = 0; j < sensorWidthValue_; j++) {
				(*indexMap)[i][j] = (maskFunc(Point {j, i}) == 0xFF) ? localIndex_t{1} : localIndex_t{0};
			}
		}
	
		// Once again scan through the GlobalLinearTopology points and accumulate the ones
		// so they become the local Indexes.
		size_t accumulation{0};
		for (size_t i = 0; i < sensorHeightValue_; i++) {
			for (size_t j = 0; j < sensorWidthValue_; j++) {
				size_t next = (*indexMap)[i][j].value();
				(*indexMap)[i][j] = localIndex_t{accumulation};
				accumulation += next;
			}
		}
		return indexMap;
	}

	// GenerateIndexLinear
	/*
	 * Creates an array of GlobalLinearIndexes, arranged by corresponding local indexes ( e.g. EquilateralTriangularTopology)
	 * Because the local indexes are array indexes there is no coorsponding local type in the assigned array.
	 */       
	template<class Space>
	template<typename localIndex_t>
	constexpr std::unique_ptr<typename GlobalLinearTopology<Space>::IndexLinearMax<typename GlobalLinearTopology<Space>::Index/*index_t*/>> 
	GlobalLinearTopology<Space>::GenerateIndexLinear(
		const IndexMap<localIndex_t> *indexMap
	){

	    using G = GlobalLinearTopology<Space>;
	    using Point = typename G::Point;
	    using Index = typename G::Index;
	    using IndexLinearMax = typename G::template IndexLinearMax<Index/*index_t*/>;

		log_verbose("[GlobalLinearTopology::GenerateIndexLinear]");
		
		log_verbose("initialize to zero");
		auto indexLinearMax = std::make_unique<IndexLinearMax>();
		std::fill(indexLinearMax->begin(), indexLinearMax->end(), Index/*index_t*/{0});
	
		log_verbose("Fill indexLinearMax");
		size_t prev{0};
		for (size_t i = 0; i < sensorHeightValue_; i++) {
			for (size_t j = 0; j < sensorWidthValue_; j++) {
				localIndex_t im = (*indexMap)[i][j];
				if (prev != im.value()) {
					// taking a Transform function that inputs a GlobalLinearTopology Point and
	                // returns a GlobalLinearTopology Index
					// That inturn gets stored in an array in local index order.
					(*indexLinearMax)[im.value()] = Transform(Point{j, i});
					
				    // Note that previous is used to prevent unnecessary Transforms and copying,
					// because the index will not change until a new value occurs
					prev = im.value();                        
				}
			}
		}
	
		return indexLinearMax;
	}
	
	//transform
	template<class Space>
	constexpr typename GlobalLinearTopology<Space>::Point 
	GlobalLinearTopology<Space>::Transform(
		GlobalLinearTopology<Space>::Index index
	){
	    using Point = typename GlobalLinearTopology<Space>::Point;
		return Point{index.value() % sensorWidthValue_, index.value() / sensorWidthValue_};
	}

	//Transform
	/*
	 */
	template<class Space>
	constexpr typename GlobalLinearTopology<Space>::Index 
	GlobalLinearTopology<Space>::Transform(
		const Point &p
	){
		
	    using G = GlobalLinearTopology<Space>;
		using Index = typename G::Index;
		
		return Index{p.y() * sensorWidthValue_ + p.x()};
	}

	//GlobalLinearTopology
	/*
	 */
	template<class Space>
	GlobalLinearTopology<Space>::GlobalLinearTopology()
	{

	    using G = GlobalLinearTopology<Space>;
	    using Point = typename G::Point;
	    using Index = typename G::Index;

		log_verbose("[GlobalLinearTopology::constructor]");

		shapeDefiningPoints_ = DefineShapeDefiningPoints();
		indexMap_ = GenerateIndexMap<Index>([this](const Point p){return Mask(p);});
		indexLinearMax_ = GenerateIndexLinear/*<Index>*/(indexMap_.get());
		assert(indexMap_->size() == sensorHeightValue_);
		assert((*indexMap_)[0].size() == sensorWidthValue_);
	 }

    //Size
    /*
     */
	template<class Space>
	size_t GlobalLinearTopology<Space>::size() {
		return (*indexMap_)[sensorHeightValue_ - 1][sensorWidthValue_ - 1].value() + 1;
	}

    //FrameBufferMask
	template<class Space>
	template<typename T0>
	std::vector<uint16_t> GlobalLinearTopology<Space>::FrameBufferMask(
		const T0 &frameBuffer, 
		const IndexLinearMax<typename GlobalLinearTopology_t::Index> *indexLinearMax, 
		size_t indexLinearSize
	){
		std::vector<uint16_t> outputData(indexLinearSize);
		for (size_t i = 0; i < indexLinearSize; i++) {
			outputData[i] = frameBuffer[(*indexLinearMax)[i].value()];
		}
		return outputData;
	}

    //FrameBufferMaskChunked
	template<class Space>
	void GlobalLinearTopology<Space>::FrameBufferMaskChunked(std::span<uint16_t> input,
									 IndexLinearMax<typename GlobalLinearTopology_t::Index>::iterator indexBegin,
									 IndexLinearMax<typename GlobalLinearTopology_t::Index>::iterator indexEnd,
									 uint16_t* outputBuffer) {
		//log_verbose("[GlobalLinearTopology::FrameBufferMaskedChunked]");
		if (indexBegin >= indexEnd) {
			std::cerr << "Invalid range: indexBegin >= indexEnd" << std::endl;
			return;
		}
		const size_t chunkSize = std::distance(indexBegin, indexEnd);
		for (size_t i = 0; i < chunkSize; ++i) {
			auto offset = indexBegin[i].value();
			if (offset >= input.size()) {
				std::cerr << "[ERROR] index " << i << " out of bounds: " << offset << " >= " << input.size() << std::endl;
				std::terminate();
			}
			outputBuffer[i] = *(input.data() + offset);
		}
	}
  
	//FrameBufferUnmask
	template<class Space>
	template<typename T0>
	std::vector<uint16_t> GlobalLinearTopology<Space>::FrameBufferUnmask(
	    const T0& frameBuffer,
	    const IndexLinearMax<typename GlobalLinearTopology_t::Index>* indexLinearMax)
	{
	    //log_verbose("[GlobalLinearTopology::FrameBufferUnmask]");
	
	    if (!indexLinearMax || frameBuffer.size() > indexLinearMax->size()) {
	        throw std::runtime_error("FrameBufferUnmask: invalid index map or size mismatch.");
	    }
	
	    std::vector<uint16_t> outputData(indexLinearMax->size(), 0);
	    FrameBufferScatterTo(frameBuffer, indexLinearMax, outputData.data(), outputData.size());
	    return outputData;
	}	
	
	//SaveArrayToFile
	template<class Space>
   template<typename index_t>  
   void 
   GlobalLinearTopology<Space>::SaveArrayToFile(
	   const IndexLinearMax<index_t> *data, 
	   const std::string& filename
   ){
		log_verbose("[GlobalLinearTopology::SaveArrayToFile]");
		std::ofstream out(filename);
		if (!out) {
			std::cerr << "Failed to open file: " << filename << std::endl;
			return;
		}
	
		for (const auto& row : *data) {
				out << row.value() << " ";
		}
	
		out.close();
		std::cout << "Saved 1536*864 array to " << filename << std::endl;
	}
	
	//SaveArrayToFile
	template<class Space>
	template<typename index_t>
	void 
	GlobalLinearTopology<Space>::SaveArrayToFile(
		const IndexMap<index_t> *data,
		const std::string& filename
	){
		log_verbose("[GlobalLinearTopology::SaveArrayToFile]");
		std::ofstream out(filename);
		if (!out) {
			std::cerr << "Failed to open file: " << filename << std::endl;
			return;
		}
	
		for (const auto& row : *data) {
			for (size_t x = 0; x < sensorWidthValue_; ++x) {
				out << row[x].value();
				if (x < sensorWidthValue_ - 1) out << " ";
			}
			out << "\n";
		}
	
		out.close();
		std::cout << "Saved 1536x864 grid to " << filename << std::endl;
	}
	
	template<class Space>
	template<typename SrcT, typename DstT>
	void GlobalLinearTopology<Space>::FrameBufferScatterTo(
	    const SrcT& frameBuffer,
	    const IndexLinearMax<typename GlobalLinearTopology<Space>::Index>* indexLinearMax,
	    DstT* dst,
	    size_t dstSize)
	{
	    //log_verbose("[GlobalLinearTopology::FrameBufferScatterTo]");
	
	    if (!indexLinearMax) {
	        throw std::runtime_error("FrameBufferScatterTo: null index map.");
	    }
	    if (frameBuffer.size() > indexLinearMax->size()) {
	        throw std::runtime_error("FrameBufferScatterTo: size mismatch.");
	    }
	    if (!dst) {
	        throw std::runtime_error("FrameBufferScatterTo: null dst.");
	    }
	
	    for (size_t i = 0; i < frameBuffer.size(); ++i) {
	        const auto outIdx = (*indexLinearMax)[i].value();
	        if (outIdx >= dstSize) {
	            throw std::runtime_error("FrameBufferScatterTo: index out of bounds.");
	        }
	        dst[outIdx] = static_cast<DstT>(frameBuffer[i]);
	    }
	}
	
	template<class Space>
	template<typename SrcT, typename DstT>
	void GlobalLinearTopology<Space>::FrameBufferScatterAccumulateTo(
	    const SrcT& frameBuffer,
	    const IndexLinearMax<typename GlobalLinearTopology<Space>::Index>* indexLinearMax,
	    DstT* dst,
	    size_t dstSize)
	{
	    log_verbose("[GlobalLinearTopology::FrameBufferScatterAccumulateTo]");
	
	    if (!indexLinearMax) {
	        throw std::runtime_error("FrameBufferScatterAccumulateTo: null index map.");
	    }
	    if (frameBuffer.size() > indexLinearMax->size()) {
	        throw std::runtime_error("FrameBufferScatterAccumulateTo: size mismatch.");
	    }
	    if (!dst) {
	        throw std::runtime_error("FrameBufferScatterAccumulateTo: null dst.");
	    }
	
	    for (size_t i = 0; i < frameBuffer.size(); ++i) {
	        const auto outIdx = (*indexLinearMax)[i].value();
	        if (outIdx >= dstSize) {
	            throw std::runtime_error("FrameBufferScatterAccumulateTo: index out of bounds.");
	        }
	        dst[outIdx] += static_cast<DstT>(frameBuffer[i]);
	    }
	}
	
	template<class Space>
	template<typename SrcT>
	void GlobalLinearTopology<Space>::FrameBufferScatterToSpan(
	    const SrcT& frameBuffer,
	    const IndexLinearMax<typename GlobalLinearTopology<Space>::Index>* indexLinearMax,
	    std::span<uint16_t> dst)
	{
	    FrameBufferScatterTo(frameBuffer, indexLinearMax, dst.data(), dst.size());
	}
	
};//ending namespace DASPi

