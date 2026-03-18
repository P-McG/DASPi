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
    MessageType type{MessageType::Unknown};
    uint16_t    version{1};
};

struct GainMsg {
    MessageHeader header{MessageType::GainMsg, 1};

    uint32_t camera_id{};
    uint32_t frame_id{};

    float mean_brightness{};
    float target_brightness{};
};

struct GainReply {
    MessageHeader header{MessageType::GainReply, 1};

    uint32_t camera_id{};
    uint32_t frame_id{};

    float requested_gain{};
    uint32_t status{}; // 0 = OK
};

static_assert(std::is_standard_layout_v<MessageHeader>);
static_assert(std::is_standard_layout_v<GainMsg>);
static_assert(std::is_standard_layout_v<GainReply>);

} // namespace DASPi
