/**
 * Copyright (c) 2025 M. Kajdak. All rights reserved.
 */

#ifndef PLATFORM_CONTROLLER_SYSCOM_RESPONSE_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_RESPONSE_HPP_

#include <platform_controller/syscom/defs/Messages.hpp>
#include <platform_controller/syscom/defs/MessageIds.hpp>

namespace platform_controller::syscom
{
struct Response
{
    union {
        PlatformSetMotorSpeedResp set_motor_speed_resp;
        PlatformSetMotorPwmValueResp set_motor_pwm_value_resp;
    } msg;
    int msg_id;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_RESPONSE_HPP_
