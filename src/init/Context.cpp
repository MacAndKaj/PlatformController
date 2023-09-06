/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */

#include <platform_controller/init/Context.hpp>

#include <platform_controller/transport/SpiProxy.hpp>

#include <stdexcept>

namespace platform_controller::init
{

void Context::setup(const std::vector<rclcpp::Parameter>& parameters)
{
    for (const auto& param : parameters)
    {
        if (param.get_name() == "transport_device_name")
        {
            m_transport_proxy= std::make_unique<transport::SpiProxy>(param.as_string());
        }
    }
    
}

transport::ITransportProxy& Context::getTransportProxy()
{
    if (!m_transport_proxy) throw std::runtime_error("Context not initialized");
    return *m_transport_proxy;
}

} // namespace platform_controller::init

