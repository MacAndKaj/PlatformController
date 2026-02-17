/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_

#include <platform_controller/init/IContext.hpp>

#include <memory>

namespace platform_controller::init
{

class Context : public IContext
{
public:
    explicit Context(rclcpp::Node& current_node);
    virtual ~Context() = default;

    void setRosCom(std::unique_ptr<roscom::IRosCom> roscom) override;
    roscom::IRosCom& getRosCom() override;
    
    void setSysCom(std::unique_ptr<syscom::ISysCom> syscom) override;
    syscom::ISysCom& getSysCom() override;
    
    void setup(const std::vector<rclcpp::Parameter>& parameters) override;
    
    transport::ITransportProxy& getTransportProxy() override;

    transport::ITransportProxy& getLogsProxy() override;

    gpio::IGpioManager& getGpioManager() override;

    ITimersManager& getTimersManager() override;

    rclcpp::Logger createLogger(const std::string& name) override;

private:
    rclcpp::Node& m_current_node;
    std::unique_ptr<roscom::IRosCom> m_roscom;
    std::unique_ptr<syscom::ISysCom> m_syscom;
    std::unique_ptr<transport::ITransportProxy> m_transport_proxy;
    std::unique_ptr<transport::ITransportProxy> m_logs_proxy;
    std::unique_ptr<gpio::IGpioManager> m_gpio_manager;
    std::unique_ptr<ITimersManager> m_timers_manager;
    bool m_syscom_debug;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_
