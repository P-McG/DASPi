#pragma once
/*
*/
namespace DASPi{
	
	class PointData {
		public:	
			size_t x_, y_;

			constexpr PointData() = default;
			constexpr PointData(size_t x, size_t y) : x_{x}, y_{y} {}
			constexpr PointData(const PointData& other) = default;
		    constexpr PointData& operator=(const PointData& other) {
		        x_ = other.x_;
		        y_ = other.y_;
		        return *this;
		    }
			constexpr bool operator==(const PointData& other) const {
				return x_ == other.x_ && y_ == other.y_;		
			}
			[[nodiscard]] constexpr size_t x() const {return x_;};
			[[nodiscard]] constexpr size_t y() const {return y_;};
	}; // Integer coordinates
	
	class DirectionData {
        public:
   			long deltaX_, deltaY_;

			constexpr DirectionData() = default;
			constexpr DirectionData(long deltaX, long deltaY) : deltaX_{deltaX}, deltaY_{deltaY} {}
			constexpr DirectionData(const DirectionData& other) = default;
			constexpr bool operator==(const DirectionData& other) const {
				return deltaX_ == other.deltaX_ &&deltaY_ == other.deltaY_;
			}
			[[nodiscard]] constexpr long deltaX() const {return deltaX_;};
			[[nodiscard]] constexpr long deltaY() const {return deltaY_;};
	};//Signed Integer coordinates
	
	class IndexData{
		public:
			using value_type = size_t;
			constexpr IndexData() = default;
			constexpr explicit IndexData(value_type index):index_{index}{};
			constexpr value_type value() const { return index_;}
			constexpr bool operator==(const IndexData& other) const { return index_ == other.index_; }
			constexpr bool operator<(const IndexData& other) const { return index_ < other.index_; }
			constexpr auto operator<=>(const IndexData&) const = default; // C++20 spaceship operator
			friend std::ostream& operator<<(std::ostream& os, const IndexData& i) {
				return os << i.index_;
			}
			    
			//using IndexData::IndexData;
		        
		private:
			value_type index_;
	};
};//ending namespace DASPi
