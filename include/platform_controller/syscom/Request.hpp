/**
  * Copyright (c) 2025 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_REQUEST_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_REQUEST_HPP_

#include <platform_controller/syscom/defs/Messages.hpp>
#include <platform_controller/syscom/defs/MessageIds.hpp>

namespace platform_controller::syscom
{

struct Request
{
    union {
        PlatformSetMotorSpeedReq set_motor_speed_req;
        PlatformSetMotorPwmValueReq set_motor_pwm_value_req;
    } msg;
    int msg_id;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_REQUEST_HPP_
 