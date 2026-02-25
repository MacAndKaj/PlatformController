/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */

#ifndef PLATFORM_CONTROLLER_GPIO_DEFS_HPP_
#define PLATFORM_CONTROLLER_GPIO_DEFS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace platform_controller::gpio
{
using LineOffset = std::uint32_t;
using LabelOffsetMap = std::unordered_map<std::string, LineOffset>;

struct ChipInfo
{
    std::string chip_dev_name;
    const std::unordered_map<std::string, LineOffset>& offsets;
};

enum class GpioInOut
{
    INPUT,
    OUTPUT,
};

enum class GpioMode
{
    EDGE_RISING,
    EDGE_FALLING,
    OPEN_DRAIN,
    OPEN_SOURCE,
};

enum class GpioOutputActiveLevel
{
    LOW,
    HIGH,
};

struct GpioConfig
{
    std::string consumer_name;
    GpioInOut inout;
    GpioOutputActiveLevel output_level;
    std::vector<GpioMode> modes;
};

struct EventExpectation
{
    unsigned int consumer_id;
    bool rising_edge;
    bool falling_edge;
    const std::string & line_name;
};

enum class GpioState
{
    ACTIVE,
    INACTIVE,
};

using LineState = std::pair<std::string, GpioState>;

} // namespace platform_controller::gpio

#endif // PLATFORM_CONTROLLER_GPIO_DEFS_HPP_