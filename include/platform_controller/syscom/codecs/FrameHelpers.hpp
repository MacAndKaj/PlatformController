/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_

#include <platform_controller/syscom/defs/Frame.hpp>

#include <vector>
#include <cstring>

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

inline Frame deserialize(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return {};
    }

    Frame frame{};
    std::memcpy(&frame, bytes.data(), FRAME_SIZE);

    return frame;
}

inline std::vector<uint8_t> serialize(const Frame& frame)
{
    std::vector<uint8_t> bytes(FRAME_SIZE);
    std::memcpy(bytes.data(), &frame, FRAME_SIZE);
    return bytes;
}

} // namespace platform_controller::syscom::codecs
 
#endif // PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_
 