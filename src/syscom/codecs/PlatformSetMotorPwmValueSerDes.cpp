/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/syscom/codecs/PlatformSetMotorPwmValueSerDes.hpp>

#include <cstring>

namespace platform_controller::syscom::codecs
{

std::vector<std::uint8_t> PlatformSetMotorPwmValueSerDes::serialize(const PlatformSetMotorPwmValueReq& msg)
{
    std::uint8_t data[DATA_SIZE];
    std::memcpy(data, &msg, sizeof(PlatformSetMotorPwmValueReq));
    return {std::begin(data), std::end(data)};}

} // namespace platform_controller::syscom::codecs
