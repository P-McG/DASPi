// software/src/common/include/DASPi-message-cast.h
#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace DASPi {

template <typename T>
const T* TryCastMessage(const void* data, std::size_t size)
{
    static_assert(std::is_standard_layout_v<T>);
    if (data == nullptr) {
        return nullptr;
    }
    if (size < sizeof(T)) {
        return nullptr;
    }
    return reinterpret_cast<const T*>(data);
}

template <typename T>
T* TryCastMessage(void* data, std::size_t size)
{
    static_assert(std::is_standard_layout_v<T>);
    if (data == nullptr) {
        return nullptr;
    }
    if (size < sizeof(T)) {
        return nullptr;
    }
    return reinterpret_cast<T*>(data);
}

} // namespace DASPi
