#pragma once

#include <array>

namespace DASPi {

class DirectionData {
public:
    using Vector = std::array<double, 3>;

    Vector direction_{};

    constexpr DirectionData() = default;

    constexpr DirectionData(Vector direction)
        : direction_{direction}
    {
    }

    constexpr DirectionData(double x, double y, double z)
        : direction_{x, y, z}
    {
    }

    constexpr DirectionData(const DirectionData& other) = default;

    constexpr DirectionData& operator=(const DirectionData& other) = default;
    
    constexpr double& operator[](std::size_t i)
	{
	    return direction_[i];
	}
	
	constexpr const double& operator[](std::size_t i) const
	{
	    return direction_[i];
	}

    [[nodiscard]] constexpr const Vector& direction() const
    {
        return direction_;
    }

    [[nodiscard]] constexpr double x() const
    {
        return direction_[0];
    }

    [[nodiscard]] constexpr double y() const
    {
        return direction_[1];
    }

    [[nodiscard]] constexpr double z() const
    {
        return direction_[2];
    }

    constexpr bool operator==(const DirectionData& other) const
    {
        return direction_ == other.direction_;
    }
};

} // namespace DASPi
