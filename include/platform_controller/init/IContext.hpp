/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_
#define PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_

#include <platform_controller/init/IRosCom.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>
#include <platform_controller/syscom/ISysCom.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init
{

class IContext
{
public:
    virtual ~IContext() = default;
    
    /// @brief Function to set RosCom object for current context.
    /// @param roscom initialized RosCom
    virtual void setRosCom(std::unique_ptr<IRosCom> roscom) = 0;

    /// @brief Returns reference to RosCom instance
    /// @return Reference to RosCom instance
    virtual IRosCom& getRosCom() = 0;
    
    /// @brief Function to set SysCom object for current context.
    /// @param roscom initialized SysCom
    virtual void setSysCom(std::unique_ptr<syscom::ISysCom> syscom) = 0;

    /// @brief Returns reference to SysCom instance
    /// @return Reference to SysCom instance
    virtual syscom::ISysCom& getSysCom() = 0;
    
    /// @brief Function to set up context with command line parameters
    /// @param parameters 
    virtual void setup(const std::vector<rclcpp::Parameter>& parameters) = 0;
    
    /// @brief Function returning reference to initialized transport proxy in current context.
    /// @return Reference to actual transport proxy.
    virtual transport::ITransportProxy& getTransportProxy() = 0;

    /// @brief Creates new logger with with node name extended with provided name.
    /// @param name String with logger name added to node prefix.
    /// @return New logger instance
    virtual rclcpp::Logger createLogger(const std::string& name) = 0;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_ICONTEXT_HPP_
