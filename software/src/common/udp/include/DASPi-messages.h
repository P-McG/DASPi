// software/src/common/include/DASPi-messages.h
#pragma once

#include <cstdint>
#include <type_traits>

namespace DASPi {

enum class MessageType : uint16_t {
    Unknown   = 0,
    Frame     = 1,
    GainMsg   = 2,
    GainReply = 3,
};

struct MessageHeader {
    MessageType type{MessageType::Unknown}; // 2
    uint16_t    version{1};                 // 2
};                                          // total 4

struct GainMsg {
    MessageHeader header;       // 4
    uint32_t camera_id;         // 4
    uint32_t frame_id;          // 4
    float mean_brightness;      // 4
    float target_brightness;    // 4
    float r_gain;               // 4
    float b_gain;               // 4
    float r_gain_apply;         // 4
    float b_gain_apply;         // 4
    float exposure_us;          // 4
    float analogue_gain;        // 4
};                              // total 44

struct GainReply {
    MessageHeader header;       // 4
    uint32_t camera_id;         // 4
    uint32_t frame_id;          // 4
    float requested_gain;       // 4
    float r_gain_apply;         // 4
    float b_gain_apply;         // 4
    uint32_t status;            // 4
};                              // total 28

static_assert(std::is_standard_layout_v<MessageHeader>);
static_assert(std::is_standard_layout_v<GainMsg>);
static_assert(std::is_standard_layout_v<GainReply>);

static_assert(std::is_trivially_copyable_v<MessageHeader>);
static_assert(std::is_trivially_copyable_v<GainMsg>);
static_assert(std::is_trivially_copyable_v<GainReply>);

static_assert(sizeof(MessageHeader) == 4);
static_assert(sizeof(GainMsg) == 44);
static_assert(sizeof(GainReply) == 28);

static_assert(alignof(GainMsg) == 4);
static_assert(alignof(GainReply) == 4);

} // namespace DASPi
