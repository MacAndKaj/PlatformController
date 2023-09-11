/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_
#define PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_

#include <platform_controller/init/IRosCom.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init
{

class IContext
{
public:
    virtual ~IContext() = default;
    
    virtual void setRosCom(std::unique_ptr<IRosCom> roscom) = 0;
    virtual IRosCom& getRosCom() = 0;
    
    virtual void setup(const std::vector<rclcpp::Parameter>& parameters) = 0;
    
    virtual transport::ITransportProxy& getTransportProxy() = 0;

    virtual rclcpp::Logger createLogger(const std::string& name) = 0;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_
