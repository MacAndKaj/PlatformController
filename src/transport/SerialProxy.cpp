/**
  * Copyright (c) 2024 M. Kajdak. All rights reserved.
  */
#include <platform_controller/transport/SerialProxy.hpp>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#include <stdexcept>

namespace platform_controller::transport
{

SerialProxy::SerialProxy(init::IContext& context, const std::string& device_path)
    : m_logger(context.createLogger("SerialProxy"))
    , m_device_path(device_path)
{
    RCLCPP_INFO(m_logger, "Creating SerialProxy");

    m_fd = open(m_device_path.c_str(), O_RDONLY);
    if (m_fd < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call open() failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        throw std::runtime_error("SerialProxy - opening error");
    }

    struct termios tty;
    try
    {
        if (tcgetattr(m_fd, &tty) != 0)
        {
            throw std::runtime_error("SerialProxy error");
        }

        tty.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        if (tcflush(m_fd, TCIFLUSH) != 0)
        {
            throw std::runtime_error("SerialProxy error");
        }

        if (tcsetattr(m_fd,TCSANOW, &tty) != 0)
        {
            throw std::runtime_error("SerialProxy error");
        }
    }
    catch (std::runtime_error&)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Configuring serial port failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        close(m_fd);
        throw;
    }
}

SerialProxy::~SerialProxy()
{
    if (m_fd >= 0)
    {
        close(m_fd);
    }
    RCLCPP_INFO(m_logger, "SerialProxy closed ");
}

bool SerialProxy::send([[maybe_unused]] const std::vector<std::uint8_t>& data)
{
    RCLCPP_ERROR(m_logger, "Method send() not implemented for SerialProxy!");
    return false;
}

std::vector<std::byte> SerialProxy::read()
{
    int bytes_in_buffer = 0;
    if (ioctl(m_fd, FIONREAD, &bytes_in_buffer) < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Cannot read available bytes. Failed with errno: (" 
            << err_status << ") "
            << std::strerror(err_status);
        RCLCPP_ERROR(m_logger, str.str().c_str());
        return {};
    }
    RCLCPP_INFO(m_logger, "Bytes in buffer %d", bytes_in_buffer);

    std::shared_ptr<std::byte[]> buffer(new std::byte[bytes_in_buffer]);
    int len  = ::read(m_fd, buffer.get(), bytes_in_buffer);
    if (len != bytes_in_buffer)
    {
        RCLCPP_WARN(m_logger, "Read less bytes than expected");
    }

    std::vector<std::byte> output;
    output.reserve(len);

    for (int i = 0; i < len; ++i)
    {
        output.emplace_back(buffer[i]);
    }

    return output;
}

} // namespace platform_controller::transport
