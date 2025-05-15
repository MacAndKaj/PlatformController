/**
  * Copyright (c) 2025 M. Kajdak. All rights reserved.
  */

#ifndef PLATFORM_CONTROLLER_GPIO_IGPIOMANAGER_HPP_
#define PLATFORM_CONTROLLER_GPIO_IGPIOMANAGER_HPP_

#include <platform_controller/gpio/defs.hpp>

namespace platform_controller::gpio
{
class IGpioManager
{
public:
    virtual ~IGpioManager() = default;
    virtual unsigned int setupLines(const std::vector<std::string>& lines, const GpioConfig& config) = 0;
    virtual bool eventOccured(const EventExpectation& expectation) = 0;
};
} // namespace platform_controller::gpio

#endif // PLATFORM_CONTROLLER_GPIO_IGPIOMANAGER_HPP_