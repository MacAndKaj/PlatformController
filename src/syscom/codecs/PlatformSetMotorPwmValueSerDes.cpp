/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/syscom/codecs/PlatformSetMotorPwmValueSerDes.hpp>

#include <cstring>

namespace platform_controller::syscom::codecs
{

std::vector<std::uint8_t> PlatformSetMotorPwmValueSerDes::serialize(const PlatformSetMotorPwmValueReq& msg)
{
    Frame frame{};
    frame.header = HEADER_BYTE;
    frame.id = PLATFORM_SET_MOTOR_PWM_VALUE_REQ_ID;
    frame.crc = 0;

    std::memcpy(frame.data, &msg, sizeof(PlatformSetMotorPwmValueReq));

    std::uint8_t frame_begin[FRAME_SIZE];
    std::memcpy(frame_begin, &frame, FRAME_SIZE);
    return {std::begin(frame_begin), std::end(frame_begin)};
}

} // namespace platform_controller::syscom::codecs
