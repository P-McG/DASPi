// DASPi-aperturepeerbase.h
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "DASPi-livecamerastate.h"

namespace DASPi{


class AperturePeerBase {
public:
    virtual ~AperturePeerBase() = default;

    virtual bool RunFrameLoop() = 0;
    virtual bool RunControlLoop() = 0;
    
    virtual bool CopyValidMask(
        std::size_t localCameraIndex,
        cv::Mat& dst
    ) const = 0;

    //virtual void InitializeCameraMasks(
        //std::vector<LiveCameraState>& liveCameras,
        //bool reverseModuleOrder
    //) = 0;
    
    virtual bool TryCopyLatestFrame(
        std::size_t localCameraIndex,
        std::vector<uint16_t>& dst
    ) const = 0;
    
    virtual bool TryShareLatestFrame(
        std::size_t localCameraIndex,
        std::shared_ptr<const std::vector<uint16_t>>& dst
    ) const = 0;

    virtual unsigned int FacetIndexValue() const = 0;
};
}//DASPi
