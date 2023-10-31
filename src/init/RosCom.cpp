/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/RosCom.hpp>

namespace platform_controller::init
{

RosCom::RosCom(rclcpp::Node& current_node)
    : m_current_node(current_node)
    , m_logger(current_node.get_logger())
{
    RCLCPP_INFO(m_logger, "Initializing RosCom");
}

Subscriber RosCom::subscribeForSetPlatformSpeed(
    const std::function<void(const motoros_interfaces::msg::SetPlatformSpeed&)>& callback)
{
    return m_current_node.create_subscription<motoros_interfaces::msg::SetPlatformSpeed>(
        "motur_head/set_platform_speed", 10, callback);
}

Subscriber RosCom::subscribeForSetPlatformPwmValue(
    const std::function<void(const motoros_interfaces::msg::SetPlatformPwmValue&)>& callback)
{
    return m_current_node.create_subscription<motoros_interfaces::msg::SetPlatformPwmValue>(
        "motur_head/set_platform_pwm_value", 10, callback);
}

} // namespace platform_controller::init
