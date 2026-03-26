// DASPi-i-camera-model.h
#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "DASPi-projection-result.h"

class ICameraModel {
public:
    virtual ~ICameraModel() = default;

	virtual bool unproject(const cv::Point2d& uv, Eigen::Vector3d& ray) const = 0;
	virtual ProjectionResult project(const Eigen::Vector3d& ray) const = 0;
	virtual cv::Size imageSize() const = 0;
};
