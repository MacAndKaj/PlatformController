/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_ITIMERSMANAGER_HPP_
#define PLATFORM_CONTROLLER_INIT_ITIMERSMANAGER_HPP_

#include <functional>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init
{
class ITimersManager
{
public:
    virtual ~ITimersManager() = default;

    /// @brief Creates new timer with provided period and callback.
    /// @param period Timer period
    /// @param callback Callback function to be called on timer expiration
    /// @return New timer ID
    virtual int startCyclicTimer(std::chrono::milliseconds period,
                                 const std::function<void()>& callback) = 0;

    /// @brief Creates new one-shot timer with provided period and callback.
    /// @param period Timer period
    /// @param callback Callback function to be called on timer expiration
    /// @return New timer ID
    virtual int startOneShotTimer(std::chrono::milliseconds period, const std::function<void()>& callback) = 0;

    /// @brief Restarts the timer with the given ID.
    /// @param id ID of the timer to restart
    virtual void restartTimer(int id) = 0;

    /// @brief Stops the timer with the given ID.
    /// @param id ID of the timer to stop
    virtual void removeTimer(int id) = 0;

    /// @brief Sets up the callback group for the timers manager.
    /// @param group Callback group to be used for the timers manager
    virtual void setupCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) = 0;
};
} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_ITIMERSMANAGER_HPP_
