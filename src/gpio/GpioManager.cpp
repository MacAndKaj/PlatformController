/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/gpio/GpioManager.hpp>

#include <platform_controller/init/IContext.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <thread>
#include <chrono>
#include <sstream>
#include <stdexcept>

#include <sys/ioctl.h>
#include <linux/gpio.h>

namespace platform_controller::gpio
{

gpio_v2_line_config createConfig(const GpioConfig& in)
{
    gpio_v2_line_config out;
    std::memset(&out, 0, sizeof(out));
    out.flags = (in.inout == GpioInOut::INPUT) ? GPIO_V2_LINE_FLAG_INPUT : GPIO_V2_LINE_FLAG_OUTPUT;

    for (const auto& mode : in.modes)
    {
        switch (mode)
        {
        case GpioMode::EDGE_RISING:
            out.flags |= GPIO_V2_LINE_FLAG_EDGE_RISING;
            break;
        case GpioMode::EDGE_FALLING:
            out.flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING;
            break;
        case GpioMode::OPEN_DRAIN:
            out.flags |= GPIO_V2_LINE_FLAG_OPEN_DRAIN;
            break;
        case GpioMode::OPEN_SOURCE:
            out.flags |= GPIO_V2_LINE_FLAG_OPEN_SOURCE;
            break;
        default:
            break;
        }
    }

    if (in.output_level == GpioOutputActiveLevel::LOW)
    {
        out.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
    }

    
    return out;
}

std::string requestToString(const struct gpio_v2_line_request& req)
{
    std::stringstream str;
    str << "Request = ";
    str << "Consumer:" << req.consumer << "|";
    str << "Num lines:" << req.num_lines << "|";
    str << "Offsets:[";
    for (size_t i = 0; i < req.num_lines; i++)
    {
        str << req.offsets[i] << " ";
    }
    str << "]|";
    str << "Config:[";
    str << "Flags:" << std::hex << req.config.flags << "|";
    str << "Num attrs:" << req.config.num_attrs << "|";
    str << "Attrs:[";
    for (size_t i = 0; i < req.config.num_attrs; i++)
    {
        str << req.config.attrs[i].attr.id << " ";
    }
    str << "]|";
    return str.str();
}

GpioManager::GpioManager(init::IContext& context,
    std::shared_ptr<ChipInfo> gpio_chip)
    : m_logger(context.createLogger("GpioManager|" + gpio_chip->chip_dev_name))
    , m_chip_fd(-1)
    , m_gpio_chip_info(std::move(gpio_chip))
{
    m_chip_fd = open(m_gpio_chip_info->chip_dev_name.c_str(), O_RDWR);
    if (m_chip_fd < 0)
    {
        int err_status = errno;
        std::stringstream str;
        str << "Call open() failed with errno: (" 
                    << err_status << ") "
                    << std::strerror(err_status) << std::endl;
        RCLCPP_ERROR(m_logger, str.str().c_str());
        throw std::runtime_error("Chip opening error");
    }
}


GpioManager::~GpioManager()
{
    close(m_chip_fd);
    for (const auto& fd : m_line_fds)
    {
        close(fd);
    }
}

unsigned int GpioManager::setupLines(const std::vector<std::string>& lines, const GpioConfig& config_blueprint)
{   
    std::string consumer = std::string{m_logger.get_name()} + "|" + config_blueprint.consumer_name;

    struct gpio_v2_line_request req;
    std::memset(&req, 0, sizeof(req));
    std::strncpy(req.consumer, consumer.c_str(), GPIO_MAX_NAME_SIZE);
    req.fd = m_chip_fd;
    req.config = createConfig(config_blueprint);
    req.num_lines = lines.size();

    for (size_t i = 0; i < lines.size(); i++)
    {
        req.offsets[i] = m_gpio_chip_info->offsets.at(lines[i]);
    }

    int lfd = ioctl(m_chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
    if (lfd < 0)
    {
        
        int err_status = errno;
        std::stringstream str;
        str << "Call ioctl(GPIO_GET_LINEHANDLE_IOCTL) failed with errno: (" 
                    << err_status << ") "
                    << std::strerror(err_status) << std::endl;
        RCLCPP_ERROR(m_logger, str.str().c_str());
        RCLCPP_ERROR(m_logger, requestToString(req).c_str());
        throw std::runtime_error("Line handle request error: ");
    }
    
    m_line_fds.emplace_back(lfd);
    return m_line_fds.size() - 1;
}

bool GpioManager::eventOccured(const EventExpectation& expectation)
{
    gpio_v2_line_event event;
    int lfd = m_line_fds.at(expectation.consumer_id);

    if (read(lfd, &event, sizeof(event)) == -1)
    {
        if (errno == -EAGAIN)
        {
            return false;
        }

        int err_status = errno;
        std::stringstream str;
        str << "Call read() for "
            << m_gpio_chip_info->chip_dev_name
            << " failed with errno: ("
            << err_status << ") "
            << std::strerror(err_status) << std::endl;
        RCLCPP_ERROR(m_logger, str.str().c_str());
        throw std::runtime_error("Event reading error");
    }

    if (((expectation.rising_edge and event.id == GPIOEVENT_EVENT_RISING_EDGE) or
        (expectation.falling_edge and event.id == GPIOEVENT_EVENT_FALLING_EDGE)) and
        m_gpio_chip_info->offsets.at(expectation.line_name) == event.offset)
    {
        return true;
    }
    
    return false;   
}

} // namespace platform_controller::gpio
