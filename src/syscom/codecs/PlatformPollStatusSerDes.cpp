/**
* Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#include <platform_controller/syscom/codecs/PlatformPollStatusSerDes.hpp>

#include <cstring>

namespace platform_controller::syscom::codecs
{

std::vector<std::uint8_t> PlatformPollStatusSerDes::serialize(const PlatformPollStatusReq& msg)
{
    Frame frame{};
    frame.header = HEADER_BYTE;
    frame.id = PLATFORM_POLL_STATUS_REQ_ID;
    frame.crc = 0;

    std::memcpy(frame.data, &msg, sizeof(PlatformPollStatusReq));

    std::uint8_t frame_begin[FRAME_SIZE];
    std::memcpy(frame_begin, &frame, FRAME_SIZE);
    return {std::begin(frame_begin), std::end(frame_begin)};
}

} // namespace platform_controller::syscom::codecs
