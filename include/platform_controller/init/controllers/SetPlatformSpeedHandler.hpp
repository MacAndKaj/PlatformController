/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMSPEEDHANDLER_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMSPEEDHANDLER_HPP_

#include <platform_controller/init/controllers/IHandler.hpp>

#include <platform_controller/init/IContext.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init::controllers
{

class SetPlatformSpeedHandler : public IHandler
{
public:
    explicit SetPlatformSpeedHandler(IContext& context);
    virtual ~SetPlatformSpeedHandler() = default;

protected:
    void handle(const motoros_interfaces::msg::SetPlatformSpeed& msg);

private:
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::SubscriptionBase> m_subscription;
};

} // namespace platform_controller::init::controllers

#endif // PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMSPEEDHANDLER_HPP_
