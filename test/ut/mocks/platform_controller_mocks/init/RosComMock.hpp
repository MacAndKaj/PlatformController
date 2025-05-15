/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_ROSCOMMOCK_HPP_
#define PLATFORM_CONTROLLER_INIT_ROSCOMMOCK_HPP_

#include <platform_controller/init/IRosCom.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace platform_controller::init
{

class RosComMock : public IRosCom
{
public:
    MOCK_METHOD(Subscriber,
        subscribeForSetPlatformSpeed,
        (const std::function<void(const motoros_interfaces::msg::SetPlatformSpeed&)>&));
    MOCK_METHOD(Subscriber,
        subscribeForSetPlatformPwmValue,
        (const std::function<void(const motoros_interfaces::msg::SetPlatformPwmValue&)>&));
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_ROSCOMMOCK_HPP_
