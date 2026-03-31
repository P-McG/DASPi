// DASPi-wire-utils.cpp
#include "DASPi-wire-utils.h"

namespace DASPi{

float HostToWireFloat(float v)
{
    const uint32_t bits = std::bit_cast<uint32_t>(v);
    return std::bit_cast<float>(htonl(bits));
}

float WireToHostFloat(float v)
{
    const uint32_t bits = std::bit_cast<uint32_t>(v);
    return std::bit_cast<float>(ntohl(bits));
}

MessageHeader ToWireMessageHeader(MessageHeader h)
{
    using MT = std::underlying_type_t<MessageType>;
    h.type = static_cast<MessageType>(
        htons(static_cast<uint16_t>(static_cast<MT>(h.type)))
    );
    h.version = htons(h.version);
    return h;
}

MessageHeader FromWireMessageHeader(MessageHeader h)
{
    using MT = std::underlying_type_t<MessageType>;
    h.type = static_cast<MessageType>(
        ntohs(static_cast<uint16_t>(static_cast<MT>(h.type)))
    );
    h.version = ntohs(h.version);
    return h;
}

GainMsg ToWireGainMsg(GainMsg g)
{
    g.header = ToWireMessageHeader(g.header);

    g.camera_id = htonl(g.camera_id);
    g.frame_id  = htonl(g.frame_id);

    g.mean_brightness   = HostToWireFloat(g.mean_brightness);
    g.target_brightness = HostToWireFloat(g.target_brightness);
    g.r_gain            = HostToWireFloat(g.r_gain);
    g.b_gain            = HostToWireFloat(g.b_gain);
    g.r_gain_apply      = HostToWireFloat(g.r_gain_apply);
    g.b_gain_apply      = HostToWireFloat(g.b_gain_apply);
    g.exposure_us       = HostToWireFloat(g.exposure_us);
    g.analogue_gain     = HostToWireFloat(g.analogue_gain);

    return g;
}

GainMsg FromWireGainMsg(GainMsg g)
{
    g.header = FromWireMessageHeader(g.header);

    g.camera_id = ntohl(g.camera_id);
    g.frame_id  = ntohl(g.frame_id);

    g.mean_brightness   = WireToHostFloat(g.mean_brightness);
    g.target_brightness = WireToHostFloat(g.target_brightness);
    g.r_gain            = WireToHostFloat(g.r_gain);
    g.b_gain            = WireToHostFloat(g.b_gain);
    g.r_gain_apply      = WireToHostFloat(g.r_gain_apply);
    g.b_gain_apply      = WireToHostFloat(g.b_gain_apply);
    g.exposure_us       = WireToHostFloat(g.exposure_us);
    g.analogue_gain     = WireToHostFloat(g.analogue_gain);

    return g;
}

FrameHeader ToWireFrameHeader(FrameHeader h)
{
    h.magic_ = htonl(h.magic_);
    h.gainMsg_ = ToWireGainMsg(h.gainMsg_);
    h.payloadSize_ = htonl(h.payloadSize_);

    for (auto& s : h.regionSizes_) {
        s = htonl(s);
    }

    h.checksum_ = htonl(h.checksum_);
    return h;
}

FrameHeader FromWireFrameHeader(FrameHeader h)
{
    h.magic_ = ntohl(h.magic_);
    h.gainMsg_ = FromWireGainMsg(h.gainMsg_);
    h.payloadSize_ = ntohl(h.payloadSize_);

    for (auto& s : h.regionSizes_) {
        s = ntohl(s);
    }

    h.checksum_ = ntohl(h.checksum_);
    return h;
}
};//namespace DASPi
