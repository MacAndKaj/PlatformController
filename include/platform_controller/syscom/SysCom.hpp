/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_

#include <platform_controller/syscom/ISysCom.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

namespace platform_controller::syscom
{

class SysCom : public ISysCom
{
public:
    explicit SysCom(init::IContext& context);
    virtual ~SysCom() = default;
    bool send(const PlatformSetMotorSpeedReq& msg) override;
    bool send(const PlatformSetMotorPwmValueReq& msg) override;

private:
    rclcpp::Logger m_logger;
    transport::ITransportProxy& m_proxy;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_
