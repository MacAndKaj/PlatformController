/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */

#include <platform_controller/syscom/SysCom.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>
#include <platform_controller/syscom/codecs/FrameHelpers.hpp>
#include <platform_controller/syscom/codecs/PlatformPollStatusSerDes.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorSpeedSerDes.hpp>
#include <platform_controller/syscom/codecs/PlatformSetMotorPwmValueSerDes.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

namespace platform_controller::syscom
{

SysCom::SysCom(init::IContext& context)
    : m_logger(context.createLogger("SysCom"))
    , m_subscriptions_counter(0)
    , m_proxy(context.getTransportProxy())
{
    RCLCPP_INFO(m_logger, "SysCom initialized");
}

void SysCom::work()
{
    if (m_subscriptions.empty())
    {
        return;
    }
    
    std::vector<std::uint8_t> bytes = m_proxy.read(FRAME_SIZE);
    if (bytes.empty())
    {
        RCLCPP_ERROR(m_logger, "Error while receiving data!");
        return;
    }
    dispatch(bytes);
}

bool SysCom::send(const Request& msg)
{
    std::vector<std::uint8_t> bytes;
    switch (msg.msg_id)
    {
    case PLATFORM_SET_MOTOR_SPEED_REQ_ID:
        bytes = codecs::PlatformSetMotorSpeedSerDes::serialize(msg.msg.set_motor_speed_req);
        break;
    case PLATFORM_SET_MOTOR_PWM_VALUE_REQ_ID:
        bytes = codecs::PlatformSetMotorPwmValueSerDes::serialize(msg.msg.set_motor_pwm_value_req);
        break;
    case PLATFORM_POLL_STATUS_REQ_ID:
        bytes = codecs::PlatformPollStatusSerDes::serialize(msg.msg.platform_poll_status_req);
        break;
    default:
        RCLCPP_ERROR(m_logger, "Error while serializing message!");
        return false;
    };

    if (not m_proxy.send(bytes))
    {
        RCLCPP_ERROR(m_logger, "Error while sendind data!");
        return false;
    }

    return true;
}

int SysCom::subscribe(MessageId msgId, const Callback& callback)
{
    return m_subscriptions.emplace(m_subscriptions_counter++, std::make_pair(msgId, callback)).first->first;
}

void SysCom::dispatch(const std::vector<std::uint8_t>& bytes)
{
    if (not codecs::frameCheck(bytes))
    {
        RCLCPP_ERROR(m_logger, "Error while checking frame!");
        return;
    }

    Response response{};
    response.msg_id = codecs::msgId(bytes);
    RCLCPP_INFO(m_logger, "Received message id: %d", response.msg_id);
    switch (response.msg_id)
    {
    case PLATFORM_SET_MOTOR_SPEED_RESP_ID:
        response.msg.set_motor_speed_resp = codecs::deserialize<PlatformSetMotorSpeedResp>(bytes);
        break;
    case PLATFORM_SET_MOTOR_PWM_VALUE_RESP_ID:
        response.msg.set_motor_pwm_value_resp = codecs::deserialize<PlatformSetMotorPwmValueResp>(bytes);
        break;
    case PLATFORM_POLL_STATUS_RESP_ID:
        response.msg.poll_status_resp = codecs::deserialize<PlatformPollStatusResp>(bytes);
        break;
    default:
        RCLCPP_ERROR(m_logger, "Error while dispatching message!");
        return;
    };

    for (const auto& subscription : m_subscriptions)
    {
        if (subscription.second.first != response.msg_id)
        {
            continue;
        }
        subscription.second.second(response);
    }
}

} // namespace platform_controller::syscom
