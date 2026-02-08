/**
* Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_CODECS_PLATFORMPOLLSTATUSSERDES_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_CODECS_PLATFORMPOLLSTATUSSERDES_HPP_

#include <cstdint>
#include <platform_controller/syscom/defs/Frame.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>
#include <vector>

namespace platform_controller::syscom::codecs
{

class PlatformPollStatusSerDes
{
public:
    virtual ~PlatformPollStatusSerDes() = default;
    static std::vector<std::uint8_t> serialize(const PlatformPollStatusReq& msg);
};

} // namespace platform_controller::syscom::codecs

#endif // PLATFORM_CONTROLLER_SYSCOM_CODECS_PLATFORMPOLLSTATUSSERDES_HPP_
