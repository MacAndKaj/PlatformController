/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTEXTMOCK_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTEXTMOCK_HPP_

#include <platform_controller/init/IContext.hpp>

#include <platform_controller/init/IRosCom.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gmock/gmock.h>

namespace platform_controller::init
{

class ContextMock : public IContext
{
public:
    MOCK_METHOD(void, setRosCom, (std::unique_ptr<IRosCom>));
    MOCK_METHOD(IRosCom&, getRosCom, ());
    MOCK_METHOD(void, setSysCom,(std::unique_ptr<syscom::ISysCom> syscom));
    MOCK_METHOD(syscom::ISysCom&, getSysCom,());
    MOCK_METHOD(void, setup, (const std::vector<rclcpp::Parameter>&));
    MOCK_METHOD(transport::ITransportProxy&, getTransportProxy, ());
    MOCK_METHOD(rclcpp::Logger, createLogger, (const std::string&));
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_CONTEXTMOCK_HPP_
