/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_COMMAND_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_COMMAND_HPP_

#include <platform_controller/syscom/defs/Messages.hpp>

namespace platform_controller::syscom
{

struct Command
{
    union {
        PlatformSetMotorSpeedReq set_motor_speed_req;
        PlatformSetMotorPwmValueReq set_motor_pwm_value_req;
        PlatformPollStatusReq platform_poll_status_req;
    } msg;
    int msg_id;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_COMMAND_HPP_
 