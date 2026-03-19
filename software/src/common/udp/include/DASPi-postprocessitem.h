//postprocessitem.h
#pragma once
#include "DASPi-messages.h"

namespace DASPi{
struct PostProcessItem {
    uint64_t frameNumber;
    libcamera::FrameBuffer *buffer;
    GainMsg gainMsg;
};
}//namespace DASPi
