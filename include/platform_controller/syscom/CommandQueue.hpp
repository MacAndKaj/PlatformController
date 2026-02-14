/**
* Copyright (c) 2026 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_COMMANDQUEUE_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_COMMANDQUEUE_HPP_

#include <platform_controller/syscom/Command.hpp>

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <list>
#include <vector>

namespace platform_controller::init
{
class IContext;
}

namespace platform_controller::syscom
{

class CommandQueue
{
public:
    explicit CommandQueue(init::IContext& context);

    void push(Command cmd);
    std::pair<std::vector<std::uint8_t>, std::uint8_t> pop();

private:
    rclcpp::Logger m_logger;
    std::list<Command> m_queue;
};

} // namespace platform_controller::syscom

#endif //PLATFORM_CONTROLLER_SYSCOM_COMMANDQUEUE_HPP_
