/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */

#include <platform_controller/init/PlatformControllerNode.hpp>

#include <platform_controller/init/controllers/SetPlatformSpeedHandler.hpp>
#include <platform_controller/init/controllers/SetPlatformPwmValueHandler.hpp>

#include <map>
#include <vector>

namespace platform_controller::init
{

static const std::map<std::string, rclcpp::ParameterType> ARGUMENTS = {
    {"transport_device_name", rclcpp::ParameterType::PARAMETER_STRING},
};

PlatformControllerNode::PlatformControllerNode(const std::string& node_name)
    : rclcpp::Node(node_name)
    , m_node_logger(get_logger())
{
    for (const auto& [key, val] : ARGUMENTS)
    {
        declare_parameter(key, val);
    }

    RCLCPP_INFO(m_node_logger, "Node initialized");
}

void PlatformControllerNode::setContext(std::shared_ptr<IContext> context)
{
    m_context = std::move(context);
    std::vector<std::string> arg_names;
    arg_names.reserve(ARGUMENTS.size());
    for (const auto& [key, val] : ARGUMENTS)
    {
         arg_names.emplace_back(key);
    }

    m_context->setup(get_parameters(arg_names));
}

void PlatformControllerNode::setup()
{
    m_controllers.emplace_back(std::make_shared<controllers::SetPlatformSpeedHandler>(*m_context));
    m_controllers.emplace_back(std::make_shared<controllers::SetPlatformPwmValueHandler>(*m_context));
}

} // namespace platform_controller::init
