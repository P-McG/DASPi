// software/src/common/include/DASPi-messages.h
#pragma once

#include <cstdint>
#include <type_traits>
#include "DASPi-message-header.h"
namespace DASPi {

struct GainReply {
    MessageHeader header;       // 4
    uint32_t camera_id;         // 4
    uint32_t frame_id;          // 4
    float requested_gain;       // 4
    float r_gain_apply;         // 4
    float b_gain_apply;         // 4
    uint32_t status;            // 4
};                              // total 28

static_assert(std::is_standard_layout_v<GainReply>);

static_assert(std::is_trivially_copyable_v<GainReply>);

static_assert(sizeof(GainReply) == 28);

static_assert(alignof(GainReply) == 4);

} // namespace DASPi
