/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */

#ifndef PLATFORM_CONTROLLER_GPIO_GPIOMANAGER_HPP_
#define PLATFORM_CONTROLLER_GPIO_GPIOMANAGER_HPP_

#include <platform_controller/gpio/IGpioManager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace platform_controller::init
{
class IContext;
}

namespace platform_controller::gpio
{
class GpioManager : public IGpioManager
{
public:
    GpioManager(init::IContext& context, std::shared_ptr<ChipInfo> gpio_chip_info);
    virtual ~GpioManager();
    unsigned int setupLines(const std::vector<std::string>& lines, const GpioConfig& config) override;
    bool eventOccured(const EventExpectation& expectation) override;
    void setLineValue(unsigned consumer_id, const std::vector<LineState>& lines_with_state) override;

private:
    rclcpp::Logger m_logger;
    int m_chip_fd{-1};
    std::shared_ptr<ChipInfo> m_gpio_chip_info;
    std::vector<int> m_line_fds;
    // Mapping: for each consumer (index matches m_line_fds), a map from line name to index inside the handle
    std::vector<std::unordered_map<std::string, unsigned int>> m_line_index_map;
};
} // namespace platform_controller::gpio

#endif // PLATFORM_CONTROLLER_GPIO_GPIOMANAGER_HPP_