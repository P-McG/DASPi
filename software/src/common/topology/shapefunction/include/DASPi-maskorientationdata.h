// DASPi-maskorientationdata.h
#pragma once
/*
*/
namespace DASPi{
	
	class MaskOrientationData {
        public:
	        double radius_;
   			double cosValue_, sinValue_;

			constexpr MaskOrientationData() = default;
			constexpr MaskOrientationData(double radius, double cosValue, double sinValue) : radius_{radius}, cosValue_{cosValue}, sinValue_{sinValue} {}
			constexpr MaskOrientationData(const MaskOrientationData& other) = default;
			constexpr bool operator==(const MaskOrientationData& other) const {
				return radius_ == other.radius_ && cosValue_ == other.cosValue_ && sinValue_ == other.sinValue_;
			}
			[[nodiscard]] constexpr double radius() const {return radius_;};
			[[nodiscard]] constexpr double cosValue() const {return cosValue_;};
			[[nodiscard]] constexpr double sinValue() const {return sinValue_;};
	};//Signed double coordinates
};//ending namespace DASPi
