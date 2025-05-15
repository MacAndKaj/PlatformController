/**
  * Copyright (c) 2024 M. Kajdak. All rights reserved.
  */

#include <platform_controller/init/UserCommunicationNode.hpp>

#include <map>
#include <vector>

namespace platform_controller::init
{

static const std::map<std::string, rclcpp::ParameterType> ARGUMENTS = {
};

UserCommunicationNode::UserCommunicationNode(const std::string& node_name)
    : rclcpp::Node(node_name)
    , m_node_logger(get_logger())
{
    for (const auto& [key, val] : ARGUMENTS)
    {
        declare_parameter(key, val);
    }

    RCLCPP_INFO(m_node_logger, "Node initialized");
}

void UserCommunicationNode::setContext(std::shared_ptr<IContext> context)
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

void UserCommunicationNode::setup()
{
    RCLCPP_INFO(m_node_logger, "Node setup");
}

} // namespace platform_controller::init
