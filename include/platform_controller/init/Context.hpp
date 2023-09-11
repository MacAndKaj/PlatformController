/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_

#include <platform_controller/init/IContext.hpp>

#include <platform_controller/init/IRosCom.hpp>

#include <memory>

namespace platform_controller::init
{

class Context : public IContext
{
public:
    explicit Context(rclcpp::Node& current_node);
    virtual ~Context() = default;

    void setRosCom(std::unique_ptr<IRosCom> roscom) override;
    IRosCom& getRosCom() override;
    
    void setup(const std::vector<rclcpp::Parameter>& parameters) override;
    
    transport::ITransportProxy& getTransportProxy() override;

    rclcpp::Logger createLogger(const std::string& name) override;

private:
    rclcpp::Node& m_current_node;
    std::unique_ptr<IRosCom> m_roscom;
    std::unique_ptr<transport::ITransportProxy> m_transport_proxy;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_
