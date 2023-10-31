/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_

#include <platform_controller/syscom/defs/Messages.hpp>

namespace platform_controller::syscom
{

class ISysCom
{
public:
    virtual ~ISysCom() = default;
    virtual bool send(const PlatformSetMotorSpeedReq& msg) = 0;
    virtual bool send(const PlatformSetMotorPwmValueReq& msg) = 0;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_
