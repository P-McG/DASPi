// DASPi-pinhole-camera-model.h
#pragma once
#include "DASPi-projection-result.h"
#include "DASPi-i-camera-model.h"

class PinholeCameraModel : public ICameraModel {
public:
    PinholeCameraModel(const cv::Matx33d& K, const cv::Size& size)
        : K_(K), size_(size)
    {
        Kinv_ = K_.inv();
    }

	bool unproject(const cv::Point2d& uv, Eigen::Vector3d& ray) const override {
	    const cv::Vec3d p(uv.x, uv.y, 1.0);
	    const cv::Vec3d q = Kinv_ * p;
	
	    ray = Eigen::Vector3d(q[0], q[1], q[2]).normalized();
	    return true;
	}

    ProjectionResult project(const Eigen::Vector3d& ray_cam) const override {
        ProjectionResult result;

        if (ray_cam.z() <= 0.0) {
            return result;
        }

        const double x = ray_cam.x() / ray_cam.z();
        const double y = ray_cam.y() / ray_cam.z();

        const cv::Vec3d p = K_ * cv::Vec3d(x, y, 1.0);
        result.uv = cv::Point2d(p[0], p[1]);

        if (result.uv.x >= 0.0 && result.uv.y >= 0.0 &&
            result.uv.x < size_.width && result.uv.y < size_.height) {
            result.valid = true;
        }

        return result;
    }

    cv::Size imageSize() const override {
        return size_;
    }

private:
    cv::Matx33d K_;
    cv::Matx33d Kinv_;
    cv::Size size_;
};
