/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGES_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGES_HPP_

#include <stdint.h>

#include "MessageIds.hpp"

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

/*
 * ONLY FOR DEBUGGING PURPOSES
 * lPwmValue - value of PWM duty for left motor
 * rPwmValue - value of PWM duty for right motor
 * lDirection - direction of left motor rotation 0-forward/0-backward
 * rDirection - direction of right motor rotation 0-forward/0-backward
 */
struct PlatformSetMotorPwmValueReq
{
    uint32_t lPwmValue;
    uint32_t rPwmValue;
    uint8_t lDirection;
    uint8_t rDirection;
};
typedef struct PlatformSetMotorPwmValueReq PlatformSetMotorPwmValueReq;

/*
 * status - status of setting the value
 */
struct PlatformSetMotorPwmValueResp
{
    enum Status status;
};
typedef struct PlatformSetMotorPwmValueResp PlatformSetMotorPwmValueResp;

/*
 * statusSet - bitset to request specific statuses
 *      bits:
 *           0 - speed
 *           1 - modules status
 */
struct PlatformPollStatusReq
{
    uint8_t statusSet; 
};
typedef struct PlatformPollStatusReq PlatformPollStatusReq;

/*
 * lSpeedF/rSpeedF  - fraction part of speed values - 0.xx
 * lSpeedI/rSpeedI  - integer part of speed values - xxx.0
 * moduleStatus     - bitset with modules Status (1-OK, 0-NOK)
 *                  bits: 0...7 - controller|feedback|log|syscom|[4...7 Not used]
 */
struct PlatformPollStatusResp
{
    int8_t lSpeedI;
    uint8_t lSpeedF; 
    int8_t rSpeedI;
    uint8_t rSpeedF;
    uint8_t moduleStatus;
};
typedef struct PlatformPollStatusResp PlatformPollStatusResp;

#endif // PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGES_HPP_
