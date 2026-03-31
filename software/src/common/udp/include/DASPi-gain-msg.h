// software/src/common/include/DASPi-messages.h
#pragma once

#include <cstdint>
#include <type_traits>
#include "DASPi-message-header.h"

namespace DASPi {

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

static_assert(std::is_standard_layout_v<GainMsg>);

static_assert(std::is_trivially_copyable_v<GainMsg>);

static_assert(sizeof(GainMsg) == 44);

static_assert(alignof(GainMsg) == 4);

} // namespace DASPi
