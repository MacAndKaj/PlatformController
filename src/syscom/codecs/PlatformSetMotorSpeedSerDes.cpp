/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/syscom/codecs/PlatformSetMotorSpeedSerDes.hpp>

#include <cstring>

namespace platform_controller::syscom::codecs
{

std::vector<std::uint8_t> PlatformSetMotorSpeedSerDes::serialize(const PlatformSetMotorSpeedReq& msg)
{
    std::uint8_t data[DATA_SIZE];
    std::memcpy(data, &msg, sizeof(PlatformSetMotorSpeedReq));
    return {std::begin(data), std::end(data)};
}

} // namespace platform_controller::syscom::codecs
