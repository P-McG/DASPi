// DASPi-livecamerastate.h
#pragma once
#include "DASPi-live-frame-buffer.h"
#include "DASPi-logicalstreamrole.h"

struct LiveCameraState {
    LiveFrameBuffer frame;
    cv::Mat validMask;
    LogicalStreamRole role{LogicalStreamRole::Overlap};
    bool hasMask{false};
};
