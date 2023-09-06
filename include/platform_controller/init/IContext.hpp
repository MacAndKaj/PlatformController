/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_
#define PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_

#include <platform_controller/transport/ITransportProxy.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init
{

class IContext
{
public:
    virtual ~IContext() = default;
    virtual void setup(const std::vector<rclcpp::Parameter>& parameters) = 0;
    virtual transport::ITransportProxy& getTransportProxy() = 0;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_
