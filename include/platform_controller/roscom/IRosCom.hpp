/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_IROSCOM_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_IROSCOM_HPP_

#include <platform_controller/roscom/IPublisherBuilder.hpp>
#include <platform_controller/roscom/IRosComSender.hpp>

// must get rid of it - requires to move interfaces to separate lib in cmake
#include <motoros_interfaces/msg/set_platform_speed.hpp>
#include <motoros_interfaces/msg/set_platform_pwm_value.hpp>
#include <motoros_interfaces/srv/platform_status_polling.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

using Subscriber = std::shared_ptr<rclcpp::SubscriptionBase>;
using Service = std::shared_ptr<rclcpp::ServiceBase>;
namespace mi_messages = motoros_interfaces::msg;
namespace mi_services = motoros_interfaces::srv;

namespace platform_controller::roscom
{

class IRosCom
{
public:
    virtual ~IRosCom() = default;
    virtual void setupSubscriptionsCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) = 0;
    virtual void setupServicesCallbackGroup(rclcpp::CallbackGroup::SharedPtr group) = 0;
    virtual Subscriber subscribeForSetPlatformSpeed(
        const std::function<void(const mi_messages::SetPlatformSpeed&)>& callback) = 0;
    virtual Subscriber subscribeForSetPlatformPwmValue(
        const std::function<void(const mi_messages::SetPlatformPwmValue&)>& callback) = 0;
    virtual Service createServiceForPollPlatformStatus(
        const std::function<void(std::shared_ptr<mi_services::PlatformStatusPolling::Request>,
                                 std::shared_ptr<mi_services::PlatformStatusPolling::Response>)>& callback) = 0;
    virtual std::shared_ptr<IRosComSender> createSender(const IPublisherBuilder& builder) = 0;
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_IROSCOM_HPP_
