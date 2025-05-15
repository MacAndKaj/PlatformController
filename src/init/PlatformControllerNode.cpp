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
    {"spi_device_name", rclcpp::ParameterType::PARAMETER_STRING},
    {"gpio_device_name", rclcpp::ParameterType::PARAMETER_STRING},
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

    using namespace std::chrono_literals;
    constexpr std::chrono::milliseconds PERIOD = 1ms;
    m_node_timer = create_wall_timer(PERIOD, [this](){
        try
        {
            slaveMonitoring();
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

void PlatformControllerNode::slaveMonitoring()
{
    m_context->getSysCom().work();
}

} // namespace platform_controller::init
