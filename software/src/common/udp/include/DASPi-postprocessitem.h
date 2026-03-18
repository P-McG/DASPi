//postprocessitem.h
#pragma once
#include "DASPi-gainmsg.h"

struct PostProcessItem {
    uint64_t frameNumber;
    libcamera::FrameBuffer *buffer;
    GainMsg gainMsg;
};
