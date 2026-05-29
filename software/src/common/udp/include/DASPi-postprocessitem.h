//postprocessitem.h
#pragma once
#include "DASPi-message-header.h"
#include "DASPi-gain-msg.h"
#include "DASPi-gain-reply.h"

namespace DASPi{
struct PostProcessItem {
    uint64_t frameNumber;
    libcamera::FrameBuffer *buffer;
    GainMsg gainMsg;
};
}//namespace DASPi
