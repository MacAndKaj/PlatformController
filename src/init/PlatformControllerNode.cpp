/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */

#include <platform_controller/init/PlatformControllerNode.hpp>

#include <map>
#include <vector>

namespace platform_controller::init
{

static const std::map<std::string, rclcpp::ParameterType> ARGUMENTS = {
    {"transport_device_name", rclcpp::ParameterType::PARAMETER_STRING},
};

PlatformControllerNode::PlatformControllerNode(const std::string& node_name, std::shared_ptr<IContext> context)
    : rclcpp::Node(node_name)
    , m_node_logger(get_logger())
    , m_context(std::move(context))
{
    RCLCPP_INFO(m_node_logger, "Setting up");

    std::vector<std::string> arg_names;
    arg_names.reserve(ARGUMENTS.size());
    for (const auto& [key, val] : ARGUMENTS)
    {
        declare_parameter(key, val);
        arg_names.emplace_back(key);
    }

    m_context->setup(get_parameters(arg_names));

    RCLCPP_INFO(m_node_logger, "Node initialized");
}



} // namespace platform_controller::init
