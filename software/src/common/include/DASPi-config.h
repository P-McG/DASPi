#pragma once
#include <cstddef>
#include <numbers>
#include <cmath>

namespace DASPi {

inline constexpr size_t NUM_SIDES   = 3;      // your n
inline constexpr size_t NUM_REGIONS = NUM_SIDES + 1;
inline constexpr double orientationValue_ = 0.5+1.0/double(NUM_SIDES);
inline constexpr double overlapScaleValue_ = 0.75;
inline constexpr double faceNormalAngle_ = std::acos(std::sqrt(5.0) / 3.0);

} // namespace DASPi}
