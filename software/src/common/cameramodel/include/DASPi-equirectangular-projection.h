// DASPi-equirectangular-projection.h
#pragma once

#include <Eigen/Dense>
#include "DASPi-spherical-math.h"

namespace DASPi{
class EquirectangularProjection {
public:
    EquirectangularProjection(int width, int height)
        : width_(width), height_(height) {}

    inline Eigen::Vector3d pixelToRay(int x, int y) const {
        return spherical::EquirectPixelToRay(x, y, width_, height_);
    }

    inline Eigen::Vector2d rayToPixel(const Eigen::Vector3d& ray) const {
        const double lambda = std::atan2(ray.x(), ray.z());
        const double phi    = std::asin(ray.y());

        const double u = (lambda + M_PI) / (2.0 * M_PI);
        const double v = 0.5 - phi / M_PI;

        return Eigen::Vector2d(u * width_, v * height_);
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    int width_;
    int height_;
};
}//DASPi
