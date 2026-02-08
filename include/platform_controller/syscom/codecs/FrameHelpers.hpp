/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_

#include <platform_controller/syscom/defs/Frame.hpp>

namespace platform_controller::syscom::codecs
{

inline bool frameCheck(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return false;
    }

    if (bytes[0] != HEADER_BYTE)
    {
        return false;
    }

    return true;
}

inline uint8_t msgId(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return 0;
    }

    return bytes[1];
}

template<typename T>
inline T deserialize(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return {};
    }

    Frame frame{};
    std::memcpy(&frame, bytes.data(), FRAME_SIZE);

    T msg;
    std::memcpy(&msg, frame.data, sizeof(T));

    return msg;
}

} // namespace platform_controller::syscom::codecs
 
#endif // PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_
 