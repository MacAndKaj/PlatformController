/**
  * Copyright (c) 2024 MacAndKaj. All rights reserved.
  */
#include <platform_controller/transport/SerialProxy.hpp>

#include <sys/ioctl.h>
#include <fcntl.h>
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
        m_previous_tty = tty;

        tty.c_iflag |= (PARMRK | INPCK);
        tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | ISTRIP | IXON);

        tty.c_cflag |= (CS8 | PARENB | CLOCAL | CREAD | CSTOPB);

	    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | NOFLSH);
        tty.c_oflag = 0;
        
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if(cfsetispeed(&tty, B115200) != 0 || cfsetospeed(&tty, B115200) != 0)
        {
            throw std::runtime_error("Setting speed failed");
        }

        if (tcsetattr(m_fd, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(m_logger, "INPUT MODE FLAG: %d", tty.c_iflag);    
            RCLCPP_ERROR(m_logger, "OUTPUT MODE FLAG: %d", tty.c_oflag);    
            RCLCPP_ERROR(m_logger, "CONTROL MODE FLAG: %d", tty.c_cflag);    
            RCLCPP_ERROR(m_logger, "LOCAL MODE FLAG: %d", tty.c_lflag);    
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
        tcsetattr(m_fd, TCSANOW, &m_previous_tty);
        close(m_fd);
    }
    RCLCPP_INFO(m_logger, "SerialProxy closed ");
}

bool SerialProxy::send([[maybe_unused]] const std::vector<std::uint8_t>& data)
{
    RCLCPP_ERROR(m_logger, "Method send() not implemented for SerialProxy!");
    return false;
}

std::vector<std::uint8_t> SerialProxy::read()
{
    constexpr size_t BUFFER_SIZE{256};
    std::uint8_t buffer[BUFFER_SIZE];
    std::vector<std::uint8_t> output;
    output.reserve(BUFFER_SIZE); // just for prealocation, logs shouldn't be longer, if slow then increase
    if (auto bytes_read = ::read(m_fd, buffer, BUFFER_SIZE); bytes_read > 0)
    {
        output.insert(output.end(), buffer, buffer + bytes_read);
    }
    return output;
}

std::vector<std::uint8_t> SerialProxy::read([[maybe_unused]]unsigned int nbytes)
{
    return std::vector<std::uint8_t>();
}

} // namespace platform_controller::transport
