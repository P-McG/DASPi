// software/src/common/include/DASPi-messages.h
#pragma once

#include <cstdint>
#include <type_traits>

#include "DASPi-message-type.h"

namespace DASPi {

struct MessageHeader {
    MessageType type{MessageType::Unknown}; // 2
    uint16_t    version{1};                 // 2
};                                          // total 4

static_assert(std::is_standard_layout_v<MessageHeader>);

static_assert(std::is_trivially_copyable_v<MessageHeader>);

static_assert(sizeof(MessageHeader) == 4);

} // namespace DASPi
