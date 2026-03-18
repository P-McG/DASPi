#pragma once
#include <array>

namespace DASPi{

	class BoundingBox{
		public:
		   std::array<size_t, 2> widthBound_;
		   std::array<size_t, 2> heightBound_;
		   
		   size_t DeltaWidth(){return (widthBound_[1]-widthBound_[0]);}
	   	   size_t DeltaHeight(){return (heightBound_[1]-heightBound_[0]);}
	   	   size_t size(){return DeltaWidth()*DeltaHeight();}
	   	   

	};
};//ending namespace pragma
