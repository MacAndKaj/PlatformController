/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/controllers/SetPlatformSpeedHandler.hpp>

#include <platform_controller/init/IContext.hpp>

namespace platform_controller::init::controllers
{

SetPlatformSpeedHandler::SetPlatformSpeedHandler(IContext& context)
    : m_logger(context.createLogger("SetPlatformSpeedHandler"))
    , m_subscription(context.getRosCom().subscribeForSetPlatformSpeed(
        std::bind(&SetPlatformSpeedHandler::handle, this, std::placeholders::_1)))
    , m_proxy(context.getTransportProxy())
{
    RCLCPP_INFO(m_logger, "SetPlatformSpeedHandler waiting for messages");
}

void SetPlatformSpeedHandler::handle(const motoros_interfaces::msg::SetPlatformSpeed& msg)
{
    RCLCPP_INFO(m_logger, "Received SetPlatformSpeed - %f[left] | %f[right]", msg.l_speed, msg.r_speed);
    if (not m_proxy.send())
    {
        RCLCPP_ERR(m_logger, "Received SetPlatformSpeed - %f[left] | %f[right]", msg.l_speed, msg.r_speed);
    }
}

} // namespace platform_controller::init::controllers
