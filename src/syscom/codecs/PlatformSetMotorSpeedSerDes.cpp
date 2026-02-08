/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/syscom/codecs/PlatformSetMotorSpeedSerDes.hpp>

#include <cstring>

namespace platform_controller::syscom::codecs
{

std::vector<std::uint8_t> PlatformSetMotorSpeedSerDes::serialize(const PlatformSetMotorSpeedReq& msg)
{
    Frame frame{};
    frame.header = HEADER_BYTE;
    frame.id = PLATFORM_SET_MOTOR_SPEED_REQ_ID;
    frame.crc = 0;

    std::memcpy(frame.data, &msg, sizeof(PlatformSetMotorSpeedReq));

    std::uint8_t frame_begin[FRAME_SIZE];
    std::memcpy(frame_begin, &frame, FRAME_SIZE);
    return {std::begin(frame_begin), std::end(frame_begin)};
}

} // namespace platform_controller::syscom::codecs
