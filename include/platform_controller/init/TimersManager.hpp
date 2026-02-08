/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_TIMERSMANAGER_HPP_
#define PLATFORM_CONTROLLER_INIT_TIMERSMANAGER_HPP_

#include <platform_controller/init/ITimersManager.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init
{

class TimersManager : public ITimersManager
{
public:
    explicit TimersManager(rclcpp::Node& current_node);
    ~TimersManager() override = default;

    int startCyclicTimer(std::chrono::milliseconds period, const std::function<void()>& callback) override;
    int startOneShotTimer(std::chrono::milliseconds period, const std::function<void()>& callback) override;
    void restartTimer(int id) override;
    void removeTimer(int id) override;
    void setupCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) override;

private:
    rclcpp::Node& m_current_node;
    rclcpp::Logger m_logger;
    int m_timer_id_counter;
    std::unordered_map<int, rclcpp::TimerBase::SharedPtr> m_timers;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_TIMERSMANAGER_HPP_
