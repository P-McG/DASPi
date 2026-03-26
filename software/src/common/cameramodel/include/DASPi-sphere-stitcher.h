// DASPi-sphere-stitcher.h
#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-i-camera-model.h"
#include "DASPi-equirectangular-projection.h"

struct CameraView {
    cv::Mat image;
    cv::Mat maskNonOverlap;
    cv::Mat maskOverlap;

    std::shared_ptr<ICameraModel> model;
    Eigen::Matrix3d Rcw;
};

struct SphereStitchConfig {
    int outputWidth = 4096;
    int outputHeight = 2048;
    double blendPower = 4.0;
    cv::Vec3b backgroundColor = {0, 0, 0};
};

struct Contribution {
    int cameraIndex = -1;
    cv::Point2d uv;
    cv::Vec3d color;
    double weight = 0.0;
    bool fromNonOverlap = false;
};

class SphereStitcher {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SphereStitcher(std::vector<CameraView> cameras,
                   SphereStitchConfig config);

	cv::Mat stitch() const;
	cv::Mat stitch(cv::Mat* validMask) const;
	cv::Mat stitchFisheye(cv::Mat* validMask = nullptr) const;

private:
    void precomputeWorldRays();
    std::size_t rayIndex(int x, int y) const;

    std::vector<Contribution> gatherContributions(const Eigen::Vector3f& ray_world) const;
    cv::Vec3b resolvePixel(const std::vector<Contribution>& contributions) const;
	cv::Vec3b renderPixel(int x, int y, std::uint8_t* valid) const;
	cv::Mat makePolygonMask(int width, int height, const std::vector<std::vector<cv::Point>>& polygons);

private:
    std::vector<CameraView> cameras_;
    SphereStitchConfig config_;
    EquirectangularProjection projection_;
    std::vector<Eigen::Vector3f> worldRays_;
};
