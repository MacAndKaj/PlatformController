/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_ROSCOM_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_ROSCOM_HPP_

#include <platform_controller/roscom/IRosCom.hpp>

namespace platform_controller::roscom
{

class RosCom : public IRosCom
{
public:
    explicit RosCom(rclcpp::Node& current_node, init::IContext& context);
    virtual ~RosCom() = default;
    void setupSubscriptionsCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) override;
    void setupServicesCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) override;
    Subscriber subscribeForSetPlatformSpeed(
        const std::function<void(const mi_messages::SetPlatformSpeed&)>& callback) override;
    Subscriber subscribeForSetPlatformPwmValue(
        const std::function<void(const mi_messages::SetPlatformPwmValue&)>& callback) override;
    Service createServiceForPollPlatformStatus(
        const std::function<void(std::shared_ptr<mi_services::PlatformStatusPolling::Request>,
                                 std::shared_ptr<mi_services::PlatformStatusPolling::Response>)>& callback) override;
    std::shared_ptr<IRosComSender> createSender(const IPublisherBuilder& builder) override;
    
private:
    rclcpp::Node& m_current_node;
    init::IContext& m_context;
    rclcpp::Logger m_logger;
    rclcpp::SubscriptionOptions m_subscriptions_options;
    rclcpp::CallbackGroup::SharedPtr m_services_callback_group;
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_ROSCOM_HPP_
