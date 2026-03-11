//postprocessitem.h
#pragma once
#include "gainmsg.h"

struct PostProcessItem {
    uint64_t frameNumber;
    libcamera::FrameBuffer *buffer;
    GainMsg gainMsg;
};
