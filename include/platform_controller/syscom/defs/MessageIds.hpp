/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGEIDS_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGEIDS_HPP_

// DEPRECATED BEGIN
//******************************************************************************
//                   Services Interface
//******************************************************************************
#define PLATFORM_SET_MOTOR_SPEED_REQ_ID       0x01  // -> PlatformSetSpeedReq
#define PLATFORM_SET_MOTOR_SPEED_RESP_ID      0x02  // -> PlatformSetSpeedResp
#define PLATFORM_SET_MOTOR_PWM_VALUE_REQ_ID   0x03  // -> PlatformSetPwmValueReq
#define PLATFORM_SET_MOTOR_PWM_VALUE_RESP_ID  0x04  // -> PlatformSetPwmValueResp
#define PLATFORM_POLL_STATUS_REQ_ID           0x05  // -> PlatformPollStatusReq
#define PLATFORM_POLL_STATUS_RESP_ID          0x06  // -> PlatformPollStatusResp
//******************************************************************************
// DEPRECATED END

//******************************************************************************
//                   New Interface
//******************************************************************************

// TODO: change values after removal of old interface
#define HEARTBEAT_MSG_ID                      0x07  // ->
#define PLATFORM_STATUS_MSG_ID                0x08  // ->
#define CMD_SET_MOTOR_SPEED_ID                0x09  // -> PlatformSetSpeedReq
#define CMD_SET_MOTOR_PWM_VALUE_ID            0x0A  // -> PlatformSetPwmValueReq

#endif //PLATFORM_CONTROLLER_SYSCOM_DEFS_MESSAGEIDS_HPP_
