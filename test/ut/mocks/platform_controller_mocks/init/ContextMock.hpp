/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_CONTEXTMOCK_HPP_
#define PLATFORM_CONTROLLER_INIT_CONTEXTMOCK_HPP_

#include <platform_controller/init/IContext.hpp>

#include <platform_controller/roscom/IRosCom.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gmock/gmock.h>

namespace platform_controller::init
{

class ContextMock : public IContext
{
public:
    MOCK_METHOD(void, setRosCom, (std::unique_ptr<roscom::IRosCom>));
    MOCK_METHOD(roscom::IRosCom&, getRosCom, ());
    MOCK_METHOD(void, setSysCom,(std::unique_ptr<syscom::ISysCom> syscom));
    MOCK_METHOD(syscom::ISysCom&, getSysCom,());
    MOCK_METHOD(void, setup, (const std::vector<rclcpp::Parameter>&));
    MOCK_METHOD(transport::ITransportProxy&, getTransportProxy, ());
    MOCK_METHOD(transport::ITransportProxy&, getLogsProxy, ());
    MOCK_METHOD(gpio::IGpioManager&, getGpioManager, ());
    MOCK_METHOD(rclcpp::Logger, createLogger, (const std::string&));
    MOCK_METHOD(ITimersManager&, getTimersManager, ());
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_CONTEXTMOCK_HPP_
