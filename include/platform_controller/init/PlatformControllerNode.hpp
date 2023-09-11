/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_
#define PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_

#include <platform_controller/init/IContext.hpp>

#include <platform_controller/init/controllers/IHandler.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace platform_controller::init
{

class PlatformControllerNode : public rclcpp::Node
{
public:
    explicit PlatformControllerNode(const std::string& node_name);
    virtual ~PlatformControllerNode() = default;
    void setContext(std::shared_ptr<IContext>);
    void setup();

private:
    rclcpp::Logger m_node_logger;
    std::shared_ptr<IContext> m_context;
    std::vector<std::shared_ptr<controllers::IHandler>> m_controllers;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_
