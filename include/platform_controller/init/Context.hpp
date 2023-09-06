/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
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
    Context() = default;
    virtual ~Context() = default;
    void setup(const std::vector<rclcpp::Parameter>& parameters) override;
    transport::ITransportProxy& getTransportProxy() override;

private:
    std::unique_ptr<transport::ITransportProxy> m_transport_proxy;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_CONTEXT_HPP_
