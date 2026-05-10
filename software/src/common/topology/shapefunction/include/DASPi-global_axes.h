// DASPi-global_axes.h
#pragma once
#include <Eigen/Dense>

namespace DASPi{
struct GlobalAxes {
    Eigen::Vector3d longitudeZero = Eigen::Vector3d::UnitX();      // lon = 0
    Eigen::Vector3d longitudePositive = Eigen::Vector3d::UnitY();  // positive lon direction
    Eigen::Vector3d north = Eigen::Vector3d::UnitZ();              // top of sphere
};
}//DASPi
