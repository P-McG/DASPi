// DASPi-pointdata2dD.h
#pragma once
/*
*/
namespace DASPi{
	
	class PointData3dD {
		public:	
			double x_, y_, z_;

			constexpr PointData3dD() = default;
			constexpr PointData3dD(double x, double y, double z) : x_{x}, y_{y}, z_{z} {}
			constexpr PointData3dD(const PointData3dD& other) = default;
		    constexpr PointData3dD& operator=(const PointData3dD& other) {
		        x_ = other.x_;
		        y_ = other.y_;
				z_ = other.z_;
		        return *this;
		    }
			constexpr bool operator==(const PointData3dD& other) const {
				return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;		
			}
			[[nodiscard]] constexpr double x() const {return x_;};
			[[nodiscard]] constexpr double y() const {return y_;};
            [[nodiscard]] constexpr double z() const {return z_;};
	}; // double coordinates
};//ending namespace DASPi
