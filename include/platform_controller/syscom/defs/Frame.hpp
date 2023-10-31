/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_DEFS_FRAME_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_DEFS_FRAME_HPP_

#include <stdint.h>

#define DATA_SIZE 125
#define HEADER_BYTE 0xF0

struct Frame
{
    uint8_t header;
    uint8_t id;
    uint8_t data[DATA_SIZE];
    uint8_t crc;
};

typedef struct Frame Frame;

#define FRAME_SIZE sizeof(Frame)

#define PLATFORM_SET_MOTOR_SPEED_REQ_ID 0x01  // -> PlatformSetSpeedReq
#define PLATFORM_SET_MOTOR_SPEED_RESP_ID 0x02  // -> PlatformSetSpeedResp
#define PLATFORM_SET_MOTOR_PWM_VALUE_REQ_ID 0x03  // -> PlatformSetPwmValueReq
#define PLATFORM_SET_MOTOR_PWM_VALUE_RESP_ID 0x04  // -> PlatformSetPwmValueResp


#endif // PLATFORM_CONTROLLER_SYSCOM_DEFS_FRAME_HPP_
