#pragma once
/*
*/
namespace DASPi{
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
