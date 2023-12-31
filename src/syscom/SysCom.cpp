/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */

#include <platform_controller/syscom/SysCom.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorSpeedSerDes.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorPwmValueSerDes.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

namespace platform_controller::syscom
{

SysCom::SysCom(init::IContext& context)
    : m_logger(context.createLogger("SysCom"))
    , m_proxy(context.getTransportProxy())
{
    RCLCPP_INFO(m_logger, "SysCom initialized");
}

bool SysCom::send(const PlatformSetMotorSpeedReq& msg)
{
    std::vector<std::uint8_t> bytes = codecs::PlatformSetMotorSpeedSerDes::serialize(msg);
    
    if (not m_proxy.send(bytes))
    {
        RCLCPP_ERROR(m_logger, "Error while sendind data!");
        return false;
    }

    return true;
}

bool SysCom::send(const PlatformSetMotorPwmValueReq& msg)
{
    std::vector<std::uint8_t> bytes = codecs::PlatformSetMotorPwmValueSerDes::serialize(msg);
    
    if (not m_proxy.send(bytes))
    {
        RCLCPP_ERROR(m_logger, "Error while sendind data!");
        return false;
    }

    return true;
}

} // namespace platform_controller::syscom
