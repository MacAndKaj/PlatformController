/**
  * Copyright (c) 2024 MacAndKaj. All rights reserved.
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
    std::vector<std::uint8_t> sendRead(const std::vector<std::uint8_t>& data) override
    {
        send(data);
        return {};
    }

    std::vector<std::uint8_t> read() override;
    std::vector<std::uint8_t> read(unsigned int nbytes) override;

private:
    rclcpp::Logger m_logger;
    const std::string m_device_path;
    int m_fd{-1};
    struct termios m_previous_tty;
};

} // namespace platform_controller::transport

#endif // PLATFORM_CONTROLLER_TRANSPORT_SERIALPROXY_HPP_
