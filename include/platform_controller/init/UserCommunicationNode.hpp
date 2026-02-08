/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_USERCOMMUNICATIONNODE_HPP_
#define PLATFORM_CONTROLLER_INIT_USERCOMMUNICATIONNODE_HPP_

#include <platform_controller/init/IContext.hpp>

#include <platform_controller/init/controllers/IHandler.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace platform_controller::init
{

  // TODO: Class for communicating with user joystick.
class UserCommunicationNode : public rclcpp::Node
{
public:
    explicit UserCommunicationNode(const std::string& node_name);
    virtual ~UserCommunicationNode();
    void setContext(std::shared_ptr<IContext>);
    void setup();

private:
    rclcpp::Logger m_node_logger;
    std::shared_ptr<IContext> m_context;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_USERCOMMUNICATIONNODE_HPP_
