/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMSPEEDHANDLER_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMSPEEDHANDLER_HPP_

#include <platform_controller/init/controllers/IHandler.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/ISysCom.hpp>

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
    syscom::ISysCom& m_syscom;
};

} // namespace platform_controller::init::controllers

#endif // PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMSPEEDHANDLER_HPP_
