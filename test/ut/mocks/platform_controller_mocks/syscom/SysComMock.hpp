/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_SYSCOMMOCK_HPP_
#define PLATFORM_CONTROLLER_INIT_SYSCOMMOCK_HPP_

#include <platform_controller/syscom/ISysCom.hpp>

#include <memory>

namespace platform_controller::syscom
{

class SysComMock : public ISysCom
{
public:
    virtual ~SysComMock() = default;
    MOCK_METHOD(void, work,());
    MOCK_METHOD(bool, send, (const Request& msg));
    MOCK_METHOD(int, subscribe, (MessageId msgId, const Callback& callback));
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_INIT_SYSCOMMOCK_HPP_
