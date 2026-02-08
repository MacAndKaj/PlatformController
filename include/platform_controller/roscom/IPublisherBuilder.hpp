/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_IPUBLISHERBUILDER_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_IPUBLISHERBUILDER_HPP_

#include <platform_controller/roscom/IRosComSender.hpp>

#include <rclcpp/rclcpp.hpp>


namespace platform_controller::init
{
class IContext;
} // namespace platform_controller::init

namespace platform_controller::roscom
{

class IPublisherBuilder
{
public:
    virtual ~IPublisherBuilder() = default;
    virtual std::shared_ptr<IRosComSender> build(init::IContext& context, rclcpp::Node& node) const = 0;
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_IPUBLISHERBUILDER_HPP_