/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_TRANSPORT_SPIPROXY_HPP_
#define PLATFORM_CONTROLLER_TRANSPORT_SPIPROXY_HPP_

#include <platform_controller/transport/ITransportProxy.hpp>

#include <platform_controller/init/IContext.hpp>

#include <rclcpp/rclcpp.hpp>

#include <linux/spi/spi.h>

#include <cstdint>
#include <vector>
#include <memory>
#include <string>

namespace platform_controller::transport
{

class SpiProxy : public ITransportProxy
{
public:
    SpiProxy(init::IContext& context, const std::string& device_path);
    virtual ~SpiProxy();
    void spi_read_reg8(std::uint8_t reg);
    bool send(const std::vector<std::uint8_t>& data) override;
    std::vector<std::byte> read() override;

private:
    rclcpp::Logger m_logger;
    int m_fd{-1};
    const std::string m_device_path;
    static constexpr int s_m_word_length_bits{8};
    static constexpr int s_m_mode{SPI_MODE_3 | SPI_3WIRE};
    static constexpr int s_m_speed{1500000};
};

} // namespace platform_controller::transport

#endif // PLATFORM_CONTROLLER_TRANSPORT_SPIPROXY_HPP_
