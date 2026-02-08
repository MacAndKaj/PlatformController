/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDER_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDER_HPP_

#include <platform_controller/roscom/IRosComSender.hpp>
#include <platform_controller/roscom/PubMsg.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace platform_controller::roscom
{

template<typename T>
class RosComSender : public IRosComSender
{
public:
    RosComSender(init::IContext& context, std::shared_ptr<rclcpp::Publisher<T>> publisher);
    virtual ~RosComSender() = default;

    void send(const PubMsg& msg) override;

private:
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::Publisher<T>> m_publisher;
};

} // namespace platform_controller::roscom

#include <platform_controller/roscom/RosComSender.tpl.hpp>

#endif // PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDER_HPP_
