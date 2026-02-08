/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */

#include <platform_controller/init/TimersManager.hpp>

namespace platform_controller::init
{

TimersManager::TimersManager(rclcpp::Node& current_node)
    : m_current_node(current_node)
    , m_logger(m_current_node.get_logger().get_child("TimersManager"))
    , m_timer_id_counter(0)
{
    RCLCPP_INFO(m_logger, "TimersManager initialized");
}

int TimersManager::startCyclicTimer(std::chrono::milliseconds period, const std::function<void()>& callback)
{
    auto [fst, snd] = m_timers.emplace(
        m_timer_id_counter++,
        m_current_node.create_wall_timer(period, callback, m_callback_group));
    if (!snd)
    {
        throw std::runtime_error("Failed to create timer");
    }
    return fst->first;
}

int TimersManager::startOneShotTimer(std::chrono::milliseconds period, const std::function<void()>& callback)
{
    auto [fst, snd] = m_timers.emplace(m_timer_id_counter, m_current_node.create_wall_timer(period,
        [this, callback, timer_id=m_timer_id_counter]()
        {
            auto timer = m_timers.at(timer_id);
            timer->cancel();
            callback();
        }, m_callback_group));
    if (!snd)
    {
        throw std::runtime_error("Failed to create timer");
    }
    ++m_timer_id_counter;
    return fst->first;
}

void TimersManager::restartTimer(int id)
{
    if (auto it = m_timers.find(id); it != m_timers.end())
    {
        it->second->reset();
        return;
    }
    RCLCPP_WARN(m_logger, "Timer with ID %d does not exist", id);
}

void TimersManager::removeTimer(const int id)
{
    m_timers.erase(id);
    RCLCPP_INFO(m_logger, "Removed timer with ID: %d", id);
}

void TimersManager::setupCallbackGroup(rclcpp::CallbackGroup::SharedPtr group)
{
    m_callback_group = std::move(group);
}

} // namespace platform_controller::init
