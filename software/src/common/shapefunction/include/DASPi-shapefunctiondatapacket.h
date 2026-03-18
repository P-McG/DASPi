#pragma once
#include <vector>
#include <array>
#include <span>

#include "DASPi-logger.h"
#include "DASPi-overlapshapefunction.h"

namespace DASPi{
	template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
	class ShapeFunctionDataPacket{
		public:
		using value_type = OverlapShapeFunction<n, center, direction, nonOverlapScale>;
		
		private:
		static constexpr size_t n_ = n;
		static constexpr PointData center_ = center;
		static constexpr DirectionData direction_ = direction;
		static constexpr double nonOverlapScale_ = nonOverlapScale;
		std::vector<uint16_t> contiguousMemory_;
		std::array<std::span<uint16_t>, n_+1> regions_;
		
		public:
		// ShapeFunctionDataPackage
		/*
		 * Utility for udp tranmission that requires uint16_t pointer of contiguous memory
		 */
		 ShapeFunctionDataPacket(const OverlapShapeFunction<n, center, direction, nonOverlapScale> &sf)
		 {
			log_verbose("[ShapeFunctonDataPacket::ShapefunctionDataPacket]");
			
			log_verbose("Setup size");
			std::array<size_t, n+1> s;
			s[0] = sf.RegularPolygonalShapeFunction<n, center, direction>::size();
			for(size_t i=0; i < n; i++)
			    s[i+1] = sf.size(i);
			
			log_verbose("Set contiguousMemory");
		    contiguousMemory_ = std::vector<uint16_t> (std::accumulate(s.begin(), s.end(), 0));

			log_verbose("Partition it into spans or views");
			regions_[0] = std::span<uint16_t>(contiguousMemory_.data(), s[0]);
			
			size_t accumulativeSize{s[0]};
			for(size_t i=0; i < n; i++){
				 regions_[i+1] = std::span<uint16_t>(contiguousMemory_.data() + accumulativeSize, s[i+1]);
				 accumulativeSize += s[i+1];
			}

			log_verbose("[ShapeFunctonDataPacket::ShapefunctionDataPacket] - End");

		 }

         //void LoadUint8tToContiguousMemory( const std::vector<uint16_t>& input){
			 //log_verbose("[ShapeFunctionDataPacket::LoadUint8tToContiguousMemory]");
			 //if (size()*sizeof(uint16_t) != input.size()) {
                //throw std::runtime_error("invalid size mismatch.");
             //}
			 //memcpy(contiguousMemory_.data(), reinterpret_cast<const uint16_t*>(input.data()), contiguousMemory_.size()*sizeof(uint16_t));
		 //}
		 
         void LoadToContiguousMemory( const std::vector<uint16_t>& input){
			 log_verbose("[ShapeFunctionDataPacket::LoadUint8tToContiguousMemory]");
			 if (size()/**sizeof(uint16_t)*/ != input.size()) {
                throw std::runtime_error("invalid size mismatch.");
             }
			 memcpy(contiguousMemory_.data(), /*reinterpret_cast<const uint16_t*>(*/input.data()/*)*/, contiguousMemory_.size()*sizeof(uint16_t));
		 }
		 
		 // operator[]
 		 /*
 		  *  for a[x] = b
 		  */
	     std::span<uint16_t>& operator[](size_t i) { 
			 return regions_[i]; 
		 }   
		 
		 // operator[]        
	     /*
	      *  for a = b[x]
	      */
	     const std::span<uint16_t>& operator[](size_t i) const {
			 return regions_[i]; 
		 }
		 
		 uint16_t* RegionsUint8t(const size_t i){
			 return reinterpret_cast<uint16_t*>(regions_[i].data());
		 }
		 
 		 size_t RegionsUint8tByteSize(const size_t i){
			 return regions_[i].size() * sizeof(uint16_t);
		 }

		 std::vector<uint16_t> ContiguousMemory(){
			 return contiguousMemory_;
		 }
		 
		 uint8_t* ContiguousMemoryUint8t(){
			 return reinterpret_cast<uint8_t*>(contiguousMemory_.data());
		 }
		 
		 // move ownership out (zero-copy)
	     std::vector<uint16_t> TakeContiguousMemory() {
	  		return std::move(contiguousMemory_); 
		 }
		 
		 size_t size(){
			 return contiguousMemory_.size();
		 }
		 
		 size_t NumberOfRegions(){
			 return n+1;
		 }
	};
};//namespace DASPi
