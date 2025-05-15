/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_ROSCOM_HPP_
#define PLATFORM_CONTROLLER_INIT_ROSCOM_HPP_

#include <platform_controller/init/IRosCom.hpp>

#include <cstdint>

namespace platform_controller::init
{

class RosCom : public IRosCom
{
public:
    explicit RosCom(rclcpp::Node& current_node);
    virtual ~RosCom() = default;
    Subscriber subscribeForSetPlatformSpeed(
        const std::function<void(const motoros_interfaces::msg::SetPlatformSpeed&)>& callback) override;
    Subscriber subscribeForSetPlatformPwmValue(
        const std::function<void(const motoros_interfaces::msg::SetPlatformPwmValue&)>& callback) override;

private:
    rclcpp::Node& m_current_node;
    rclcpp::Logger m_logger;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_ROSCOM_HPP_
