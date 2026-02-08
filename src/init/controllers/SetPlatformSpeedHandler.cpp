/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/init/controllers/SetPlatformSpeedHandler.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/syscom/Request.hpp>

#include <cmath>

namespace platform_controller::init::controllers
{

SetPlatformSpeedHandler::SetPlatformSpeedHandler(IContext& context)
    : m_logger(context.createLogger("SetPlatformSpeedHandler"))
    , m_subscription(context.getRosCom().subscribeForSetPlatformSpeed(
        std::bind(&SetPlatformSpeedHandler::handle, this, std::placeholders::_1)))
    , m_syscom(context.getSysCom())
{
    RCLCPP_INFO(m_logger, "SetPlatformSpeedHandler waiting for messages");
}

void SetPlatformSpeedHandler::handle(const motoros_interfaces::msg::SetPlatformSpeed& msg)
{
    RCLCPP_INFO(m_logger, "Received SetPlatformSpeed - %f[left] | %f[right]", msg.l_speed, msg.r_speed);
    syscom::Request syscom_msg{};
    auto& req = syscom_msg.msg.set_motor_speed_req;
    syscom_msg.msg_id = PLATFORM_SET_MOTOR_SPEED_REQ_ID;

    float integer_part;
    float float_part = std::modf(msg.l_speed, &integer_part);
    req.lSpeedF = static_cast<std::uint8_t>(std::fabs(std::round(float_part*100))); 
    req.lSpeedI = static_cast<std::int8_t>(integer_part); 

    float_part = std::modf(msg.r_speed, &integer_part);
    req.rSpeedF = static_cast<std::uint8_t>(std::fabs(std::round(float_part*100))); 
    req.rSpeedI = static_cast<std::int8_t>(integer_part); 

    if (not m_syscom.send(syscom_msg))
    {
        RCLCPP_ERROR(m_logger, "Error while sendind data to platform!");
    }
    RCLCPP_INFO(m_logger, "SetPlatformSpeed sent.");
    // RCLCPP_DEBUG(m_logger, "SetPlatformSpeed sent.");
}

} // namespace platform_controller::init::controllers
