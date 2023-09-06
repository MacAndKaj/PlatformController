/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_
#define PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_

#include <platform_controller/init/IContext.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace platform_controller::init
{

class PlatformControllerNode : public rclcpp::Node
{
public:
    PlatformControllerNode(const std::string& node_name, std::shared_ptr<IContext> context);
    virtual ~PlatformControllerNode() = default;
    void setup();

private:
    rclcpp::Logger m_node_logger;

    std::shared_ptr<IContext> m_context;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_
