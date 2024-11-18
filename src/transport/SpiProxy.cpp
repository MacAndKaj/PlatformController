/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/transport/SpiProxy.hpp>

#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <iomanip>
#include <cstring>
#include <iostream>
#include <bitset>

namespace platform_controller::transport
{

std::string bufferToStr(const std::vector<std::uint8_t>& buffer)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    
    for (auto& it : buffer)
    {
        ss << "0x";
        ss << std::setw(2) << static_cast<int>(it);
        ss << " ";
    }

    return ss.str();
}

SpiProxy::SpiProxy(init::IContext& context, const std::string& device_path)
    : m_logger(context.createLogger("SpiProxy"))
    , m_device_path(device_path)
{
    m_fd = open(m_device_path.c_str(), O_RDWR);
    if (m_fd < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call open() failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        throw std::runtime_error("SpiProxy - opening error");
    }

    if (ioctl(m_fd, SPI_IOC_WR_MODE, &s_m_mode) < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call ioctl(SPI_IOC_WR_MODE) failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        close(m_fd);
        throw std::runtime_error("SpiProxy error");
    }

    if (ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &s_m_word_length_bits) < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call ioctl(SPI_IOC_WR_BITS_PER_WORD) failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        close(m_fd);
        throw std::runtime_error("SpiProxy error");
    }

    if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &s_m_speed) < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call ioctl(SPI_IOC_WR_MAX_SPEED_HZ) failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        close(m_fd);
        throw std::runtime_error("SpiProxy error");
    }
    RCLCPP_INFO(m_logger, "SpiProxy initialized");
}

SpiProxy::~SpiProxy()
{
    if (m_fd >= 0)
    {
        close(m_fd);
    }
    RCLCPP_INFO(m_logger, "SpiProxy closed ");
}

void SpiProxy::spi_read_reg8(std::uint8_t reg)
{
    std::uint8_t cmd = (1 << 7) | (reg & 0x3F); // ADXL345 READ BIT & REGISTER
    constexpr int buffer_size{1};
    // __u8 miso[2] = {0x00, 0x00};
    __u8 mosi[2] = {cmd,0x00};

    struct spi_ioc_transfer spi_transfer_buffer[buffer_size] = {
        {
            .tx_buf=(unsigned long)(mosi),
            .rx_buf=(unsigned long)NULL,
            .len=2,
            .speed_hz=s_m_speed,
            .delay_usecs=0,
            .bits_per_word=s_m_word_length_bits,
            .cs_change=0,
            .tx_nbits=0,
            .rx_nbits=0,
            .word_delay_usecs=0,
            .pad=0,
        }
    };

    int ret = ioctl(m_fd, SPI_IOC_MESSAGE(buffer_size), &spi_transfer_buffer);
    if (ret < 0)
    {
        int err_status = errno;
        std::cout   << "Call ioctl(SPI_IOC_MESSAGE) failed with errno: (" 
                    << err_status << ") "
                    << std::strerror(err_status) << std::endl;
        return;
    }
    
    std::stringstream str;
    // std::cout << "MISO: " << std::bitset<8>(miso[0]) << " - " << std::bitset<8>(miso[1]) << std::endl;
    // std::cout << "MOSI: " << std::bitset<8>(mosi[0]) << " - " << std::bitset<8>(mosi[1])  << std::endl;
    // std::cout << "Returned: " << ret << " buffer: " << str.str() << std::endl;
}

bool SpiProxy::send(const std::vector<std::uint8_t>& data)
{
    constexpr int buffer_size{1};
    std::vector<std::uint8_t> miso_buffer(data.size(), 0);

    RCLCPP_INFO(m_logger, bufferToStr(data).c_str());

    const __u8* mosi = data.data();
    const __u8* miso = miso_buffer.data();
    struct spi_ioc_transfer spi_transfer_buffer = {
        .tx_buf=(unsigned long)(mosi),
        .rx_buf=(unsigned long)(miso),
        .len=static_cast<__u32>(data.size()),
        .speed_hz=s_m_speed,
        .delay_usecs=0,
        .bits_per_word=s_m_word_length_bits,
        .cs_change=0,
        .tx_nbits=0,
        .rx_nbits=0,
        .word_delay_usecs=0,
        .pad=0,
    };
    int ret = ioctl(m_fd, SPI_IOC_MESSAGE(buffer_size), &spi_transfer_buffer);
    if (ret < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call ioctl(SPI_IOC_MESSAGE) failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        return false;
    }
    return true;    
}

std::vector<std::byte> SpiProxy::read()
{
    // not implemented
    return {};
}

} // namespace platform_controller::transport
