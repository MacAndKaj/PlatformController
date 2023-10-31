/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/controllers/SetPlatformPwmValueHandler.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>

#include <cmath>

namespace platform_controller::init::controllers
{

SetPlatformPwmValueHandler::SetPlatformPwmValueHandler(IContext& context)
    : m_logger(context.createLogger("SetPlatformPwmValueHandler"))
    , m_subscription(context.getRosCom().subscribeForSetPlatformPwmValue(
        std::bind(&SetPlatformPwmValueHandler::handle, this, std::placeholders::_1)))
    , m_syscom(context.getSysCom())
{
    RCLCPP_INFO(m_logger, "SetPlatformPwmValueHandler waiting for messages");
}

void SetPlatformPwmValueHandler::handle(const motoros_interfaces::msg::SetPlatformPwmValue& msg)
{
    RCLCPP_INFO(m_logger, "Received SetPlatformPwmValue - %d[left] | %d[right]", msg.l_pwm, msg.r_pwm);
    PlatformSetMotorPwmValueReq req{};

    req.lPwmValue = std::abs(msg.l_pwm);
    req.rPwmValue = std::abs(msg.r_pwm);
    req.lDirection = (msg.l_pwm > 0) ? 0 : 1;
    req.rDirection = (msg.r_pwm > 0) ? 0 : 1;

    if (not m_syscom.send(req))
    {
        RCLCPP_ERROR(m_logger, "Error while sendind data to platform!");
    }
}

} // namespace platform_controller::init::controllers
