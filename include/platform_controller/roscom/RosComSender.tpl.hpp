/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDER_TPL_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDER_TPL_HPP_

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/roscom/IRosComSender.hpp>
#include <platform_controller/roscom/PubMsg.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace platform_controller::roscom
{

template<typename T>
RosComSender<T>::RosComSender(init::IContext& context, std::shared_ptr<rclcpp::Publisher<T>> publisher)
    : m_logger(context.createLogger(std::string{"RosComSender/"} + publisher->get_topic_name()))
    , m_publisher(std::move(publisher))
{
}
    
template<typename T>
void RosComSender<T>::send(const PubMsg& msg)
{
    if (not std::holds_alternative<T>(msg))
    {
        RCLCPP_ERROR(m_logger, "Invalid message type");
        return;
    }

    auto message = std::get<T>(msg);
    m_publisher->publish(message);
}

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDER_TPL_HPP_
