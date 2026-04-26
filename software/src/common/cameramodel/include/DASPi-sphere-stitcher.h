// DASPi-sphere-stitcher.h
#pragma once

#include <vector>
#include <memory>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-i-camera-model.h"
#include "DASPi-equirectangular-projection.h"
#include "DASPi-camera-view.h"
#include "DASPi-sphere-stitch-config.h"
#include "DASPi-contribution.h"
#include "DASPi-rig-data.h"

class SphereStitcher {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SphereStitcher(std::vector<CameraView> cameras,
                   SphereStitchConfig config,
                   const RigData<3>& rig);

    void setCameras(const std::vector<CameraView>& cameras);

    cv::Mat stitch() const;
    cv::Mat stitch(cv::Mat* validMask) const;
	cv::Mat stitchFisheye(cv::Mat* validMask = nullptr) const;
	
    cv::Mat stitchProjectionOnlyFast(cv::Mat* validMask = nullptr) const;

    static bool IsInsideMask(const cv::Mat& mask, const cv::Point2d& uv);

private:
    void precomputeWorldRays();
    void precomputeProjectionOnlyMap();
	
    std::size_t rayIndex(int x, int y) const;

    int FindOwningFace(const Eigen::Vector3f& ray_world_f) const;
    int FindSeamLocalEdgeForRay(const Eigen::Vector3d& ray_world,
                                int faceIndex) const;

    std::vector<Contribution> gatherContributions(const Eigen::Vector3f& ray_world) const;
    cv::Vec3b resolvePixel(const std::vector<Contribution>& contributions) const;
    cv::Vec3b renderPixel(int x, int y, std::uint8_t* valid) const;

    cv::Mat makePolygonMask(int width,
                            int height,
                            const std::vector<std::vector<cv::Point>>& polygons);


private:
    struct ProjectionMapEntry {
        int cameraIndex = -1;
        float u = 0.0f;
        float v = 0.0f;
    };

private:
    std::vector<CameraView> cameras_;
    SphereStitchConfig config_;
    EquirectangularProjection projection_;
    std::vector<Eigen::Vector3f> worldRays_;
    RigData<3> rig_;
    std::vector<int> faceToCameraIndex_;

    std::vector<ProjectionMapEntry> projectionOnlyMap_;
};
