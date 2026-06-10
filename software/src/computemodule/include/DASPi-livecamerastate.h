// DASPi-livecamerastate.h
#pragma once
#include "DASPi-live-frame-buffer.h"
#include "DASPi-logicalstreamrole.h"

struct LiveCameraState {
    LiveFrameBuffer frame;
    cv::Mat validMask;

    LogicalStreamRole role{LogicalStreamRole::Overlap};

    bool hasMask{false};

    /*
     * Region ownership / blending metadata.
     *
     * region 0:
     *   blendEligible = false
     *
     * regions 1..n:
     *   blendEligible = true only if this overlap edge is adjacent
     *   to another active module face.
     */
    int moduleIndex{-1};
    int faceIndex{-1};
    int localStreamIndex{-1};
    int localEdgeIndex{-1};
    int neighborFaceIndex{-1};
    int edgeIndex{-1};

    bool blendEligible{false};
};
