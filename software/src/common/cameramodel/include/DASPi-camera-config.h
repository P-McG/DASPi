// DASPi-camera-config.h
#pragma once

#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-image-rotation.h"
namespace DASPi{
struct CameraConfig {
    std::string name;
    std::string device = "";
    std::string sourceName;

    ImageRotation imageRotation{ImageRotation::None};

    // Physical camera pose: shared by all logical streams derived
    // from the same module-facing image.
    Eigen::Matrix3d Rcw{Eigen::Matrix3d::Identity()};
    
    // Extra local-axis roll correction around camera +Z.
    double localRollDeg{0.0};

    // Which physical module produced this logical stream.
    int moduleIndex{-1};

    // Optional sensor-validity mask.
    cv::Mat sensorValidMask{};

    // Owning spherical face for this physical camera/module.
    // For derived overlap streams, this remains the module's face.
    int faceIndex{-1};

    // Logical-stream topology:
    //   localStreamIndex == 0  -> non-overlap interior
    //   localStreamIndex >= 1  -> overlap stream
    int localStreamIndex{-1};

    // For overlap streams, which local edge of faceIndex this stream corresponds to.
    // For non-overlap, keep -1.
    int localEdgeIndex{-1};

    // For overlap streams, which neighboring face lies across that edge.
    // For non-overlap, keep -1.
    int neighborFaceIndex{-1};

    // Global topology edge index for the seam associated with this overlap stream.
    // For non-overlap, keep -1.
    int edgeIndex{-1};
   
    bool seamDirectionReversed{false};
    
    bool IsNonOverlap() const
    {
        return localEdgeIndex < 0;
    }

    bool IsOverlap() const
    {
        return localEdgeIndex >= 0;
    }
};
}//DASPi
