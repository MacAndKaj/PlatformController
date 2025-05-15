/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_IROSCOM_HPP_
#define PLATFORM_CONTROLLER_INIT_IROSCOM_HPP_

// must get rid of it - requires to move interfaces to separate lib in cmake
#include <motoros_interfaces/msg/set_platform_speed.hpp>
#include <motoros_interfaces/msg/set_platform_pwm_value.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace platform_controller::init
{

using Subscriber = std::shared_ptr<rclcpp::SubscriptionBase>;

class IRosCom
{
public:
    virtual ~IRosCom() = default;
    virtual Subscriber subscribeForSetPlatformSpeed(
        const std::function<void(const motoros_interfaces::msg::SetPlatformSpeed&)>& callback) = 0;
    virtual Subscriber subscribeForSetPlatformPwmValue(
        const std::function<void(const motoros_interfaces::msg::SetPlatformPwmValue&)>& callback) = 0;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_IROSCOM_HPP_
