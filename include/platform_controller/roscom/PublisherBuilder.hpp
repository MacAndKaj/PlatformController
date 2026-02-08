/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_PUBLISHERBUILDER_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_PUBLISHERBUILDER_HPP_

#include <platform_controller/roscom/IPublisherBuilder.hpp>

#include <platform_controller/roscom/RosComSender.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::roscom
{

template<typename T>
class PublisherBuilder : public IPublisherBuilder
{
public:
    explicit PublisherBuilder(const std::string& topic)
        : m_topic(topic)
    {
    }

    virtual ~PublisherBuilder() = default;

    std::shared_ptr<IRosComSender> build(init::IContext& context, rclcpp::Node& node) const override
    {
        return std::make_shared<RosComSender<T>>(context, node.create_publisher<T>(m_topic, rclcpp::ParametersQoS()));
    }

private:
    std::string m_topic;
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_PUBLISHERBUILDER_HPP_