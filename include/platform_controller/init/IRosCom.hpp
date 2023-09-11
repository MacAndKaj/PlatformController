/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_IROSCOM_HPP_
#define PLATFORM_CONTROLLER_INIT_IROSCOM_HPP_

#include <motoros_interfaces/msg/set_platform_speed.hpp>

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
        const std::function<void(const motoros_interfaces::msg::SetPlatformSpeed&)>& callback) const = 0;
    
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_IROSCOM_HPP_
