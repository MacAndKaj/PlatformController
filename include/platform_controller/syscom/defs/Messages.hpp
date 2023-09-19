/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGES_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGES_HPP_

#include <stdint.h>

enum Status
{
    NoError,
    UndefinedError
};

/*
 * lSpeedF/rSpeedF - fraction part of speed values - 0.xxxx
 * lSpeedI/rSpeedI - integer part of speed values - xxx.0
 */
struct PlatformSetMotorSpeedReq
{
    int8_t lSpeedI;
    uint8_t lSpeedF; // max 2 numbers after comma
    int8_t rSpeedI;
    uint8_t rSpeedF; // max 2 numbers after comma
};
typedef struct PlatformSetMotorSpeedReq PlatformSetMotorSpeedReq;

struct PlatformSetMotorSpeedResp
{
    enum Status status;
};
typedef struct PlatformSetMotorSpeedResp PlatformSetMotorSpeedResp;

#endif // PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGES_HPP_
