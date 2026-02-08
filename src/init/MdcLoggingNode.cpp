/**
  * Copyright (c) 2024 MacAndKaj. All rights reserved.
  */

#include <algorithm>
#include <iterator>
#include <platform_controller/init/MdcLoggingNode.hpp>

#include <chrono>

namespace platform_controller::init
{

static const std::map<std::string, rclcpp::ParameterType> MDC_LOGGING_NODE_ARGUMENTS = {
    {"serial_device_name", rclcpp::ParameterType::PARAMETER_STRING},
};

MdcLoggingNode::MdcLoggingNode(const std::string& node_name)
    : rclcpp::Node(node_name)
    , m_node_logger(get_logger())
{
    for (const auto& [key, val] : MDC_LOGGING_NODE_ARGUMENTS)
    {
        declare_parameter(key, val);
    }

    RCLCPP_INFO(m_node_logger, "Node initialized");
}

MdcLoggingNode::~MdcLoggingNode()
{
    RCLCPP_INFO(m_node_logger, "Node destructed");
}

void MdcLoggingNode::setContext(std::shared_ptr<IContext> context)
{
    m_context = std::move(context);
    std::vector<std::string> arg_names;
    arg_names.reserve(MDC_LOGGING_NODE_ARGUMENTS.size());
    for (const auto& [key, val] : MDC_LOGGING_NODE_ARGUMENTS)
    {
         arg_names.emplace_back(key);
    }

    m_context->setup(get_parameters(arg_names));
}

void MdcLoggingNode::setup()
{
    using namespace std::chrono_literals;
    constexpr std::chrono::milliseconds PERIOD = 30ms;
    m_node_timer = create_wall_timer(PERIOD, [this](){
        try
        {
            work();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(m_node_logger, e.what());
        }
        catch(...)
        {
            RCLCPP_ERROR(m_node_logger, "Unknown exception catched");
        }
    });
}

void MdcLoggingNode::work()
{
    auto bytes = m_context->getLogsProxy().read();
    if (bytes.empty()) return;

    for (auto&& c : bytes)
    {
        m_log_queue.emplace_back(static_cast<char>(c));
    }

    // RCLCPP_INFO(m_node_logger, std::string(m_log_queue.begin(), m_log_queue.end()).c_str());

    auto it = m_log_queue.begin();
    auto new_line = std::find(m_log_queue.begin(), m_log_queue.end(), '\n');
    while (new_line != m_log_queue.end())
    {
        RCLCPP_INFO(m_node_logger, std::string(it, new_line).c_str());
        it = new_line+1; // skip new line character
        new_line = std::find(it, m_log_queue.end(), '\n');
    }

    m_log_queue.erase(m_log_queue.begin(), it);
}

} // namespace platform_controller::init
