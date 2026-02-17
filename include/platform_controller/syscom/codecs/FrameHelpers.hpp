/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_

#include <platform_controller/syscom/defs/Frame.hpp>

#include <vector>
#include <cstring>

#include <boost/crc.hpp>

namespace platform_controller::syscom::codecs
{

using crc_8_smbus = boost::crc_optimal<8,       // width 8 bits
                                       0x07,    // polynomial 0x07 (x^8 + x^2 + x + 1)
                                       0x00,    // initial value
                                       0x00,    // final XOR value
                                       false,   // reflect input bytes
                                       false>;  // reflect output CRC

inline bool frameCheck(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return false;
    }

    if (bytes[0] != HEADER_BYTE)
    {
        return false;
    }

    return true;
}

inline bool crcCheck(const Frame& frame)
{
    crc_8_smbus crc8;
    crc8.process_bytes(&frame, sizeof(frame) - sizeof(frame.crc));
    return crc8.checksum() == frame.crc;
}

inline bool addCrc(Frame& frame)
{
    crc_8_smbus crc8;
    crc8.process_bytes(&frame, sizeof(frame) - sizeof(frame.crc));
    frame.crc = crc8.checksum();
    if (frame.crc == 0)
    {
        return false;
    }
    return true;
}

inline uint8_t msgId(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return 0;
    }

    return bytes[1];
}

inline Frame deserialize(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.size() != FRAME_SIZE)
    {
        return {};
    }

    Frame frame{};
    std::memcpy(&frame, bytes.data(), FRAME_SIZE);

    return frame;
}

inline std::vector<uint8_t> serialize(const Frame& frame)
{
    std::vector<uint8_t> bytes(FRAME_SIZE);
    std::memcpy(bytes.data(), &frame, FRAME_SIZE);
    return bytes;
}

} // namespace platform_controller::syscom::codecs
 
#endif // PLATFORM_CONTROLLER_SYSCOM_CODECS_FRAMEHELPERS_HPP_
 