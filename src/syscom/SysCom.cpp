/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */

#include <ranges>
#include <platform_controller/syscom/SysCom.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>
#include <platform_controller/syscom/codecs/FrameHelpers.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorSpeedSerDes.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorPwmValueSerDes.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

#include "platform_controller/syscom/defs/MessageIds.hpp"

namespace platform_controller::syscom
{

PlatformStatus get_status(const Frame& frame)
{
    if (frame.id != PLATFORM_STATUS_MSG_ID)
    {
        return {};
    }

    PlatformStatus status{};
    std::memcpy(&status, frame.data, sizeof(PlatformStatus));
    return status;
}


SysCom::SysCom(init::IContext& context)
    : m_debug_mode(false)
    , m_logger(context.createLogger("SysCom"))
    , m_proxy(context.getTransportProxy())
    , m_command_queue(context)
    , m_connection_status(ConnectionStatus::WAITING_FOR_CONNECTION)
{
    RCLCPP_INFO(m_logger, "SysCom initialized");
}

void SysCom::work()
{
    auto [payload, cmd_id] = m_command_queue.pop();
    if (payload.empty())
    {
        payload = codecs::serialize(create_next_heartbeat_frame());
    }
    else
    {
        payload = codecs::serialize(create_next_frame(payload, cmd_id));
    }
    const auto bytes = m_proxy.sendRead(payload);
    if (not codecs::frameCheck(bytes))
    {
        if (not m_debug_mode)
        {
            RCLCPP_ERROR(m_logger, "Received frame with invalid format");
        }
        m_connection_status = ConnectionStatus::CONNECTION_LOST;
        return;
    }

    const auto frame = codecs::deserialize(bytes);
    if (not codecs::crcCheck(frame))
    {
        if (not m_debug_mode)
        {
            RCLCPP_ERROR(m_logger, "Received frame with invalid CRC: %02x", frame.crc);
        }
        return;
    }

    m_connection_status = ConnectionStatus::CONNECTED;
    handle_received_frame(frame);
}

void SysCom::send(const Command& msg)
{
    m_command_queue.push(msg);
}

int SysCom::subscribeForStatus(const Callback& callback)
{
    return m_subscriptions.emplace(m_subscriptions_counter++, callback).first->first;
}

void SysCom::setDebug(bool enabled)
{
    RCLCPP_INFO(m_logger, "Setting debug mode to: %s", enabled ? "enabled" : "disabled");
    m_debug_mode = enabled;
}

Frame SysCom::create_next_frame(const std::vector<std::uint8_t>& payload, const std::uint8_t id) const
{
    Frame frame;
    frame.header = HEADER_BYTE;
    frame.id = id;
    std::memcpy(frame.data, payload.data(), payload.size());
    codecs::addCrc(frame);
    return frame;
}

Frame SysCom::create_next_heartbeat_frame() const
{
    //TODO: create heartbeat frame with correct payload
    return create_next_frame(std::vector<std::uint8_t>(DATA_SIZE, 0), HEARTBEAT_MSG_ID);
}

void SysCom::handle_received_frame(const Frame& frame)
{
    switch (frame.id)
    {
    case PLATFORM_STATUS_MSG_ID:
        {
            auto status = get_status(frame);
            for (const auto& callback : m_subscriptions | std::views::values)
            {
                callback(status);
            }
        }
        break;
    default:
        RCLCPP_ERROR(m_logger, "Unknown message ID received: %d", frame.id);
    }
}

} // namespace platform_controller::syscom
