#pragma once
/*
*/
namespace DASPi{
	
	class PointData2dI {
		public:	
			size_t x_, y_;

			constexpr PointData2dI() = default;
			constexpr PointData2dI(size_t x, size_t y) : x_{x}, y_{y} {}
			constexpr PointData2dI(const PointData2dI& other) = default;
		    constexpr PointData2dI& operator=(const PointData2dI& other) {
		        x_ = other.x_;
		        y_ = other.y_;
		        return *this;
		    }
			constexpr bool operator==(const PointData2dI& other) const {
				return x_ == other.x_ && y_ == other.y_;		
			}
			[[nodiscard]] constexpr size_t x() const {return x_;};
			[[nodiscard]] constexpr size_t y() const {return y_;};
	}; // Integer coordinates
};//ending namespace DASPi
