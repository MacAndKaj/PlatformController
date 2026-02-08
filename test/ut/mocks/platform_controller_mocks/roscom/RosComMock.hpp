/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_ROSCOMMOCK_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_ROSCOMMOCK_HPP_

#include <platform_controller/roscom/IRosCom.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace platform_controller::roscom
{

class RosComMock : public IRosCom
{
public:
    MOCK_METHOD(Subscriber,
        subscribeForSetPlatformSpeed,
        (const std::function<void(const mi_messages::SetPlatformSpeed&)>&));
    MOCK_METHOD(Subscriber,
        subscribeForSetPlatformPwmValue,
        (const std::function<void(const mi_messages::SetPlatformPwmValue&)>&));
    MOCK_METHOD(Service,
        createServiceForPollPlatformStatus,
        (const std::function<void(std::shared_ptr<mi_services::PlatformStatusPolling::Request>,
                                  std::shared_ptr<mi_services::PlatformStatusPolling::Response>)>&));
    MOCK_METHOD(std::shared_ptr<IRosComSender>, createSender, (const IPublisherBuilder& builder));
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_ROSCOMMOCK_HPP_
