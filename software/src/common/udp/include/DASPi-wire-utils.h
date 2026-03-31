// DASPi-wire-utils.h
#pragma once
#include <bit>
#include <type_traits>
#include <arpa/inet.h>

#include "DASPi-frameheader.h"
#include "DASPi-messages.h"

namespace DASPi {

float HostToWireFloat(float v);
float WireToHostFloat(float v);

MessageHeader ToWireMessageHeader(MessageHeader h);
MessageHeader FromWireMessageHeader(MessageHeader h);

GainMsg ToWireGainMsg(GainMsg g);
GainMsg FromWireGainMsg(GainMsg g);

FrameHeader ToWireFrameHeader(FrameHeader h);
FrameHeader FromWireFrameHeader(FrameHeader h);

} // namespace DASPi
