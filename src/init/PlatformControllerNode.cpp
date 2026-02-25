/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */

#include <platform_controller/init/PlatformControllerNode.hpp>

#include <platform_controller/gpio/IGpioManager.hpp>
#include <platform_controller/gpio/Rpi3BPlusGpioChips.hpp>
#include <platform_controller/init/controllers/SetPlatformSpeedHandler.hpp>
#include <platform_controller/init/controllers/SetPlatformPwmValueHandler.hpp>
#include <platform_controller/init/services/PlatformStatusPollingService.hpp>

#include <map>
#include <vector>

namespace platform_controller::init
{

static const std::map<std::string, rclcpp::ParameterType> ARGUMENTS = {
    {"spi_device_name", rclcpp::ParameterType::PARAMETER_STRING},
    {"gpio_device_name", rclcpp::ParameterType::PARAMETER_STRING},
    {"syscom_debug", rclcpp::ParameterType::PARAMETER_BOOL},
};

PlatformControllerNode::PlatformControllerNode(const std::string& node_name)
    : Node(node_name)
    , m_node_logger(get_logger())
    , m_main_callback_group(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , m_controllers_callback_group(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , m_services_callback_group(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
    , m_timers_callback_group(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
{
    for (const auto& [key, val] : ARGUMENTS)
    {
        declare_parameter(key, val);
    }

    RCLCPP_INFO(m_node_logger, "Node initialized");
}

PlatformControllerNode::~PlatformControllerNode()
{
    RCLCPP_INFO(m_node_logger, "Node destructed");
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
    m_context->getTimersManager().setupCallbackGroup(m_timers_callback_group);
}

void PlatformControllerNode::setup()
{
    m_controllers.emplace_back(std::make_shared<controllers::SetPlatformSpeedHandler>(*m_context));
    m_controllers.emplace_back(std::make_shared<controllers::SetPlatformPwmValueHandler>(*m_context));

    m_services.emplace_back(std::make_shared<services::PlatformStatusPollingService>(*m_context));

    auto& gpio_manager = m_context->getGpioManager();
    auto config = gpio::GpioConfig{
        .consumer_name = "PlatformControllerNode",
        .inout = gpio::GpioInOut::OUTPUT,
        .output_level = gpio::GpioOutputActiveLevel::LOW,
        .modes = {}
    };
    m_gpio_consumer_id = gpio_manager.setupLines({"GPIO21"}, config);
    resetMotorDriverCard();

    using namespace std::chrono_literals;
    constexpr auto PERIOD = 10ms;

    m_node_timer = create_wall_timer(PERIOD, [this](){ syscomMasterWork(); }, m_main_callback_group);
}

void PlatformControllerNode::syscomMasterWork()
{
    try
    {
        m_context->getSysCom().work();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(m_node_logger, e.what());
    }
    catch(...)
    {
        RCLCPP_ERROR(m_node_logger, "Unknown exception catched");
    }

}

void PlatformControllerNode::resetMotorDriverCard()
{
    if (not m_gpio_consumer_id)
    {
        RCLCPP_ERROR(m_node_logger, "GPIO consumer not initialized - MDC reset not possible");
        throw std::runtime_error("GPIO consumer not initialized");
    }
    auto& gpio_manager = m_context->getGpioManager();
    gpio::LineState reset_line_state{"GPIO21", gpio::GpioState::ACTIVE};

    RCLCPP_INFO(m_node_logger, "Performing MDC reset");
    gpio_manager.setLineValue(*m_gpio_consumer_id, {reset_line_state});

    using namespace std::chrono_literals;
    rclcpp::sleep_for(1ms);

    reset_line_state.second = gpio::GpioState::INACTIVE;
    gpio_manager.setLineValue(*m_gpio_consumer_id, {reset_line_state});

    RCLCPP_INFO(m_node_logger, "MDC reset done");
}
} // namespace platform_controller::init
