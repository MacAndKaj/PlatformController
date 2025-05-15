/**
  * Copyright (c) 2024 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_TRANSPORT_SERIALPROXY_HPP_
#define PLATFORM_CONTROLLER_TRANSPORT_SERIALPROXY_HPP_

#include <platform_controller/transport/ITransportProxy.hpp>

#include <platform_controller/init/IContext.hpp>

#include <rclcpp/rclcpp.hpp>

#include <termios.h>

namespace platform_controller::transport
{

class SerialProxy : public ITransportProxy
{
public:
    SerialProxy(init::IContext& context, const std::string& device_path);
    virtual ~SerialProxy();
    bool send(const std::vector<std::uint8_t>& data) override;
    std::vector<std::uint8_t> read();
    std::vector<std::uint8_t> read(unsigned int nbytes);

private:
    rclcpp::Logger m_logger;
    const std::string m_device_path;
    int m_fd{-1};
    struct termios m_previous_tty;
};

} // namespace platform_controller::transport

#endif // PLATFORM_CONTROLLER_TRANSPORT_SERIALPROXY_HPP_
