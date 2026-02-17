/*
 *  Copyright (c) 2026 MacAndKaj. All rights reserved.
 */

#include <platform_controller/syscom/CommandQueue.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorSpeedSerDes.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorPwmValueSerDes.hpp>
#include <platform_controller/syscom/defs/MessageIds.hpp>

namespace platform_controller::syscom
{

CommandQueue::CommandQueue(init::IContext& context)
    : m_logger(context.createLogger("CommandQueue"))
{
}

void CommandQueue::push(Command cmd)
{
    m_queue.emplace_back(std::move(cmd));
    RCLCPP_INFO(m_logger, "Command queue size: %zu", m_queue.size());
}

std::pair<std::vector<std::uint8_t>, std::uint8_t> CommandQueue::pop()
{
    if (m_queue.empty())
    {
        return {{}, 0};
    }

    auto [msg, msg_id] = m_queue.front();
    m_queue.pop_front();

    switch (msg_id)
    {
    case CMD_SET_MOTOR_SPEED_ID:
        return {codecs::PlatformSetMotorSpeedSerDes::serialize(msg.set_motor_speed_req), msg_id};
    case CMD_SET_MOTOR_PWM_VALUE_ID:
        return {codecs::PlatformSetMotorPwmValueSerDes::serialize(msg.set_motor_pwm_value_req), msg_id};
    default:
        RCLCPP_ERROR(m_logger, "Error while serializing message!");
    };

    return {{}, 0};
}

} // namespace platform_controller::syscom