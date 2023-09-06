/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include "PinController.hpp"

#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#include <iostream>
#include <cstring>

PinController::PinController(const std::string& device_path)
    : m_fd(-1)
    , m_device_path(device_path)
{
    m_fd = open(m_device_path.c_str(), O_RDWR);
    if (m_fd < 0)
    {
        int err_status = errno;
        std::cout   << "Call open() failed with errno: (" 
                    << err_status << ") "
                    << std::strerror(err_status) << std::endl;
        throw std::runtime_error("SpiProxy - opening error");
    }
}

PinController::~PinController()
{
    if (m_fd >= 0)
    {
        close(m_fd);
    }
}

void PinController::setupAsOutput(const std::string& label)
{
    int label_size = label.size() > GPIO_MAX_NAME_SIZE ? GPIO_MAX_NAME_SIZE : label.size();
	
    struct gpiohandle_request pin_setup_request;
    pin_setup_request.lineoffsets[0] = 8; // Number of pin?
    pin_setup_request.flags = GPIOHANDLE_REQUEST_OUTPUT;
    pin_setup_request.lines = 1;
    std::memset(pin_setup_request.default_values, 0, GPIOHANDLES_MAX);
    label.copy(pin_setup_request.consumer_label, label_size);

    if (ioctl(m_fd, GPIO_GET_LINEHANDLE_IOCTL, &pin_setup_request) < 0) 
    {
        int err_status = errno;
        std::cout   << "Call ioctl(GPIO_GET_LINEHANDLE_IOCTL) failed with errno: (" 
                    << err_status << ") "
                    << std::strerror(err_status) << std::endl;
        throw std::runtime_error("PinController - ioctl error");
	}
}

void PinController::set()
{
    
}

void PinController::unset()
{

}

