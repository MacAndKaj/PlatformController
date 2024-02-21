/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */

#include <platform_controller/init/Context.hpp>

#include <platform_controller/transport/SpiProxy.hpp>
#include <platform_controller/transport/SerialProxy.hpp>

#include <stdexcept>

namespace platform_controller::init
{

Context::Context(rclcpp::Node& current_node)
    : m_current_node(current_node)
{
}

void Context::setRosCom(std::unique_ptr<IRosCom> roscom)
{
    m_roscom = std::move(roscom);
}

IRosCom& Context::getRosCom()
{
    if (!m_roscom) throw std::runtime_error("Context not initialized - set RosCom");
    return *m_roscom;
}

void Context::setSysCom(std::unique_ptr<syscom::ISysCom> syscom)
{
    m_syscom = std::move(syscom);
}

syscom::ISysCom& Context::getSysCom()
{
    if (!m_syscom) throw std::runtime_error("Context not initialized - set SysCom");
    return *m_syscom;
}

void Context::setup(const std::vector<rclcpp::Parameter>& parameters)
{
    for (const auto& param : parameters)
    {
        if (param.get_name() == "spi_device_name")
        {
            RCLCPP_INFO(m_current_node.get_logger(), "Creating SpiProxy");
            m_transport_proxy = std::make_unique<transport::SpiProxy>( *this, param.as_string());
        }

        if (param.get_name() == "serial_device_name")
        {
            RCLCPP_INFO(m_current_node.get_logger(), "Creating SerialProxy");
            m_logs_proxy = std::make_unique<transport::SerialProxy>( *this, param.as_string());
        }
    }
}

transport::ITransportProxy& Context::getTransportProxy()
{
    if (!m_transport_proxy) throw std::runtime_error("Context not initialized - set Transport Proxy");
    return *m_transport_proxy;
}

transport::ITransportProxy& Context::getLogsProxy()
{
    if (!m_logs_proxy) throw std::runtime_error("Context not initialized - set Logs Proxy");
    return *m_logs_proxy;
}

rclcpp::Logger Context::createLogger(const std::string& name)
{
    return m_current_node.get_logger().get_child(name);
}

} // namespace platform_controller::init

