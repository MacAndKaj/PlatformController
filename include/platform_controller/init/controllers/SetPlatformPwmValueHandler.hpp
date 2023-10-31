/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMPWMVALUEHANDLER_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMPWMVALUEHANDLER_HPP_

#include <platform_controller/init/controllers/IHandler.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/ISysCom.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init::controllers
{

class SetPlatformPwmValueHandler : public IHandler
{
public:
    explicit SetPlatformPwmValueHandler(IContext& context);
    virtual ~SetPlatformPwmValueHandler() = default;

protected:
    void handle(const motoros_interfaces::msg::SetPlatformPwmValue& msg);

private:
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::SubscriptionBase> m_subscription;
    syscom::ISysCom& m_syscom;
};

} // namespace platform_controller::init::controllers

#endif // PLATFORM_CONTROLLER_INIT_CONTROLLERS_SETPLATFORMPWMVALUEHANDLER_HPP_
