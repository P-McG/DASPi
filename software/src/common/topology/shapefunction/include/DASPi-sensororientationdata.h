#pragma once
/*
*/
namespace DASPi{
	
	class SensorOrientationData {
        public:
   			double cosValue_, sinValue_;

			constexpr SensorOrientationData() = default;
			constexpr SensorOrientationData(double cosValue, double sinValue) : cosValue_{cosValue}, sinValue_{sinValue} {}
			constexpr SensorOrientationData(const SensorOrientationData& other) = default;
			constexpr bool operator==(const SensorOrientationData& other) const {
				return cosValue_ == other.cosValue_ && sinValue_ == other.sinValue_;
			}
			[[nodiscard]] constexpr double cosValue() const {return cosValue_;};
			[[nodiscard]] constexpr double sinValue() const {return sinValue_;};
	};//Signed Integer coordinates
};//ending namespace DASPi
