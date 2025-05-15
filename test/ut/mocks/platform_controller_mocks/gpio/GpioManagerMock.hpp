/**
  * Copyright (c) 2025 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_GPIOMANAGERMOCK_HPP_
#define PLATFORM_CONTROLLER_INIT_GPIOMANAGERMOCK_HPP_
 
#include <platform_controller/gpio/IGpioManager.hpp>

#include <gmock/gmock.h>

namespace platform_controller::gpio
{

class GpioManagerMock : public IGpioManager
{
public:
    MOCK_METHOD(unsigned int,
                setupLines,
                (const std::vector<std::string>& lines, const GpioConfig& config));
    MOCK_METHOD(bool, eventOccured, (const EventExpectation& expectation));
};

} // namespace platform_controller::gpio

#endif // PLATFORM_CONTROLLER_INIT_GPIOMANAGERMOCK_HPP_
