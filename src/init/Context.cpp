/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */

#include <platform_controller/init/Context.hpp>

#include <platform_controller/gpio/GpioManager.hpp>
#include <platform_controller/gpio/Rpi3BPlusGpioChips.hpp>
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
            // TODO: change m_logs_proxy to m_transport_proxy because of 1node=1transport rule
            RCLCPP_INFO(m_current_node.get_logger(), "Creating SerialProxy");
            m_logs_proxy = std::make_unique<transport::SerialProxy>( *this, param.as_string());
        }

        if (param.get_name() == "gpio_device_name")
        {
            RCLCPP_INFO(m_current_node.get_logger(), "Creating GpioManager");

            if (param.as_string() == "chip0")
            {
                m_gpio_manager = std::make_unique<gpio::GpioManager>(*this, gpio::rpi3bplus::buildGpioChip0Info());

            }
            else if (param.as_string() == "chip1")
            {
                m_gpio_manager = std::make_unique<gpio::GpioManager>(*this, gpio::rpi3bplus::buildGpioChip1Info());
            }
            else
            {
                throw std::runtime_error("Unknown gpio chip name");
            }
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

gpio::IGpioManager& Context::getGpioManager()
{
    if (!m_gpio_manager) throw std::runtime_error("Context not initialized - set Gpio Manager");
    return *m_gpio_manager;
}

rclcpp::Logger Context::createLogger(const std::string &name)
{
    return m_current_node.get_logger().get_child(name);
}

} // namespace platform_controller::init

