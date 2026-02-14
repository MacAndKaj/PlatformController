/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_DEFS_FRAME_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_DEFS_FRAME_HPP_

#include <stdint.h>

#define DATA_SIZE 125
#define HEADER_BYTE 0xF0

struct Frame
{
    uint8_t header;
    // TODO: add sequence number
    uint8_t id;
    uint8_t data[DATA_SIZE];
    uint8_t crc;
};

typedef struct Frame Frame;

#define FRAME_SIZE sizeof(Frame)

#endif // PLATFORM_CONTROLLER_SYSCOM_DEFS_FRAME_HPP_
