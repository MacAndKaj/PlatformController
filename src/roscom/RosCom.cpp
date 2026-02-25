/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/roscom/RosCom.hpp>

#include "platform_controller/init/IContext.hpp"

namespace platform_controller::roscom
{

RosCom::RosCom(rclcpp::Node& current_node, init::IContext& context)
    : m_current_node(current_node)
    , m_context(context)
    , m_logger(current_node.get_logger())
{
    RCLCPP_INFO(m_logger, "Initializing RosCom");
}

void RosCom::setupSubscriptionsCallbackGroup(rclcpp::CallbackGroup::SharedPtr group)
{
    m_subscriptions_options.callback_group = std::move(group);
}

void RosCom::setupServicesCallbackGroup(rclcpp::CallbackGroup::SharedPtr group)
{
    m_services_callback_group = std::move(group);
}

Subscriber RosCom::subscribeForSetPlatformSpeed(
    const std::function<void(const mi_messages::SetPlatformSpeed&)>& callback)
{
    return m_current_node.create_subscription<mi_messages::SetPlatformSpeed>(
        "motur_head/set_platform_speed", 10, callback, m_subscriptions_options);
}

Subscriber RosCom::subscribeForSetPlatformPwmValue(
    const std::function<void(const mi_messages::SetPlatformPwmValue&)>& callback)
{
    return m_current_node.create_subscription<mi_messages::SetPlatformPwmValue>(
        "motur_head/set_platform_pwm_value", rclcpp::QoS{2}.best_effort(), callback, m_subscriptions_options);
}


Service RosCom::createServiceForPollPlatformStatus(
    const std::function<void(std::shared_ptr<mi_services::PlatformStatusPolling::Request>,
                             std::shared_ptr<mi_services::PlatformStatusPolling::Response>)>& callback)
{
    return m_current_node.create_service<mi_services::PlatformStatusPolling>(
        "motur_head/platform_status_polling", callback, rmw_qos_profile_services_default, m_services_callback_group);
}

std::shared_ptr<IRosComSender> RosCom::createSender(const IPublisherBuilder &builder)
{
    return builder.build(m_context, m_current_node);
}

} // namespace platform_controller::roscom
