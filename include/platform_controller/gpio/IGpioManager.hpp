/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
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
    /// @brief Sets up lines for given configuration. Returns consumer id to be used for event expectation.
    ///
    /// @param lines - vector of line names as strings to be set up
    /// @param config - configuration to be applied to all lines
    /// @return consumer id to be used for event expectation or control
    virtual unsigned int setupLines(const std::vector<std::string>& lines, const GpioConfig& config) = 0;

    /// @brief Checks if event described by expectation occurred. Should be used after setupLines() with returned consumer id.
    ///
    /// @param expectation - struct describing expected event
    /// @return true if event occurred, false otherwise
    virtual bool eventOccured(const EventExpectation& expectation) = 0;


    /// @brief Sets line value for given consumer id. Should be used after setupLines() with returned consumer id.
    ///
    /// @param consumer_id - id returned by setupLines() describing line group to set value for
    /// @param lines_with_state - vector of line names with their desired state to be set
    virtual void setLineValue(unsigned int consumer_id, const std::vector<LineState>& lines_with_state) = 0;
};
} // namespace platform_controller::gpio

#endif // PLATFORM_CONTROLLER_GPIO_IGPIOMANAGER_HPP_