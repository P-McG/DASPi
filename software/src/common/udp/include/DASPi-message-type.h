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
static_assert(sizeof(MessageType) == 2);

} // namespace DASPi
