#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <cmath>

#include "DASPi-i-camera-model.h"
#include "DASPi-projection-result.h"

class FisheyeCameraModel : public ICameraModel {
public:
    FisheyeCameraModel(double fx,
                       double fy,
                       double cx,
                       double cy,
                       double maxImageRadiusPx,
                       cv::Size size)
        : fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          maxImageRadiusPx_(maxImageRadiusPx),
          size_(size) {}

    ProjectionResult project(const Eigen::Vector3d& ray) const override {
        ProjectionResult result{};
        const Eigen::Vector3d r = ray.normalized();

        const double theta = std::acos(std::clamp(r.z(), -1.0, 1.0));
        const double sinTheta = std::sqrt(r.x() * r.x() + r.y() * r.y());

        const double scale = (sinTheta > 1e-12) ? (theta / sinTheta) : 1.0;

        const double xn = r.x() * scale;
        const double yn = r.y() * scale;

        const double u = fx_ * xn + cx_;
        const double v = cy_ - fy_ * yn;

        result.uv = cv::Point2d(u, v);

        const double du = u - cx_;
        const double dv = v - cy_;
        const double imageRadius = std::sqrt(du * du + dv * dv);

        result.valid =
            (u >= 0.0 && u < static_cast<double>(size_.width) &&
             v >= 0.0 && v < static_cast<double>(size_.height) &&
             imageRadius <= maxImageRadiusPx_);

        return result;
    }

    bool unproject(const cv::Point2d& uv, Eigen::Vector3d& ray) const override {
        const double du = uv.x - cx_;
        const double dv = uv.y - cy_;
        const double imageRadius = std::sqrt(du * du + dv * dv);

        if (imageRadius > maxImageRadiusPx_) {
            return false;
        }

        const double x = du / fx_;
        const double y = (cy_ - uv.y) / fy_;

        const double r = std::sqrt(x * x + y * y);

        if (r < 1e-12) {
            ray = Eigen::Vector3d(0.0, 0.0, 1.0);
            return true;
        }

        const double theta = r;
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);
        const double scale = sinTheta / r;

        ray = Eigen::Vector3d(
            x * scale,
            y * scale,
            cosTheta
        ).normalized();

        return true;
    }

    cv::Size imageSize() const override {
        return size_;
    }

private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double maxImageRadiusPx_;
    cv::Size size_;
};
