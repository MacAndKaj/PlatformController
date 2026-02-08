/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */

#ifndef PLATFORM_CONTROLLER_GPIO_RPI3BPLUSGPIOCHIPS_HPP_
#define PLATFORM_CONTROLLER_GPIO_RPI3BPLUSGPIOCHIPS_HPP_

#include <platform_controller/gpio/defs.hpp>

#include <memory>

namespace platform_controller::gpio::rpi3bplus
{
/*
    Created basing on gpioinfo output from RPi3B+.
    The GPIO chip 0 has 54 lines, the GPIO chip 1 has 7 lines.
*/

static const std::unordered_map<std::string, LineOffset> GpioChip0Offsets = {
    {"ID_SDA", 0},
    {"ID_SCL", 1},
    {"SDA1", 2},
    {"SCL1", 3},
    {"GPIO_GCLK", 4},
    {"GPIO5", 5},
    {"GPIO6", 6},
    {"SPI_CE1_N", 7},
    {"SPI_CE0_N", 8},
    {"SPI_MISO", 9},
    {"SPI_MOSI", 10},
    {"SPI_SCLK", 11},
    {"GPIO12", 12},
    {"GPIO13", 13},
    {"TXD1", 14},
    {"RXD1", 15},
    {"GPIO16", 16},
    {"GPIO17", 17},
    {"GPIO18", 18},
    {"GPIO19", 19},
    {"GPIO20", 20},
    {"GPIO21", 21},
    {"GPIO22", 22},
    {"GPIO23", 23},
    {"GPIO24", 24},
    {"GPIO25", 25},
    {"GPIO26", 26},
    {"GPIO27", 27},
    {"HDMI_HPD_N", 28},
    {"STATUS_LED_G", 29},
    {"CTS0", 30},
    {"RTS0", 31},
    {"TXD0", 32},
    {"RXD0", 33},
    {"SD1_CLK", 34},
    {"SD1_CMD", 35},
    {"SD1_DATA0", 36},
    {"SD1_DATA1", 37},
    {"SD1_DATA2", 38},
    {"SD1_DATA3", 39},
    {"PWM0_OUT", 40},
    {"PWM1_OUT", 41},
    {"ETH_CLK", 42},
    {"WIFI_CLK", 43},
    {"SDA0", 44},
    {"SCL0", 45},
    {"SMPS_SCL", 46},
    {"SMPS_SDA", 47},
    {"SD_CLK_R", 48},
    {"SD_CMD_R", 49},
    {"SD_DATA0_R", 50},
    {"SD_DATA1_R", 51},
    {"SD_DATA2_R", 52},
    {"SD_DATA3_R", 53}
};

static const std::unordered_map<std::string, LineOffset> GpioChip1Offsets = {
    {"BT_ON", 0},
    {"WL_ON", 1},
    {"PWR_LED_R", 2},
    {"LAN_RUN", 3},
    {"NC", 4},
    {"CAM_GPIO0", 5},
    {"CAM_GPIO1", 6}
};

auto buildGpioChip0Info()
{
    return  std::make_shared<ChipInfo>(
        "/dev/gpiochip0",
        GpioChip0Offsets
    );
}

auto buildGpioChip1Info()
{
    return std::make_shared<ChipInfo>(
        "/dev/gpiochip1",
        GpioChip1Offsets
    );
}

} // namespace platform_controller::gpio::rpi3bplus

#endif // PLATFORM_CONTROLLER_GPIO_RPI3BPLUSGPIOCHIPS_HPP_