/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_
#define PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_

#include <platform_controller/init/IContext.hpp>

#include <platform_controller/init/controllers/IHandler.hpp>
#include <platform_controller/init/services/IService.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace platform_controller::init
{

class PlatformControllerNode : public rclcpp::Node
{
public:
    explicit PlatformControllerNode(const std::string& node_name);
    virtual ~PlatformControllerNode();
    void setContext(std::shared_ptr<IContext>);
    void setup();

private:
    void syscomMasterWork();
    void resetMotorDriverCard();

    rclcpp::Logger m_node_logger;
    std::shared_ptr<IContext> m_context;
    std::vector<std::shared_ptr<controllers::IHandler>> m_controllers;
    std::vector<std::shared_ptr<services::IService>> m_services;
    rclcpp::TimerBase::SharedPtr m_node_timer;
    rclcpp::CallbackGroup::SharedPtr m_main_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_controllers_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_services_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_timers_callback_group;
    std::optional<unsigned int> m_gpio_consumer_id;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_PLATFORMCONTROLLERNODE_HPP_
