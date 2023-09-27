/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_CODECS_PLATFORMSETMOTORSPEEDSERDES_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_CODECS_PLATFORMSETMOTORSPEEDSERDES_HPP_

#include <cstdint>
#include <platform_controller/syscom/defs/Frame.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>
#include <vector>

namespace platform_controller::syscom::codecs
{

class PlatformSetMotorSpeedSerDes
{
public:
    virtual ~PlatformSetMotorSpeedSerDes() = default;
    static std::vector<std::uint8_t> serialize(const PlatformSetMotorSpeedReq& msg);
};

} // namespace platform_controller::syscom::codecs

#endif // PLATFORM_CONTROLLER_SYSCOM_CODECS_PLATFORMSETMOTORSPEEDSERDES_HPP_
