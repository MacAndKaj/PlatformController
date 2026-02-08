/**
* Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDERMOCK_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDERMOCK_HPP_

#include <platform_controller/roscom/IRosComSender.hpp>

#include <gmock/gmock.h>

#include <memory>


namespace platform_controller::roscom
{

class RosComSenderMock : public IRosComSender
{
public:
    virtual ~RosComSenderMock() = default;
    MOCK_METHOD(void, send, (const PubMsg& msg), (override));
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_ROSCOMSENDERMOCK_HPP_
