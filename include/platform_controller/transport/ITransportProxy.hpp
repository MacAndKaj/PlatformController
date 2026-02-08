/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_TRANSPORT_ITRANSPORTPROXY_HPP_
#define PLATFORM_CONTROLLER_TRANSPORT_ITRANSPORTPROXY_HPP_

#include <cstdint>
#include <vector>

namespace platform_controller::transport
{

class ITransportProxy
{
public:
    virtual ~ITransportProxy() = default;
    virtual bool send(const std::vector<std::uint8_t>& data) = 0;
    virtual std::vector<std::uint8_t> read() = 0;
    virtual std::vector<std::uint8_t> read(unsigned int nbytes) = 0;
};

} // namespace platform_controller::transport

#endif // PLATFORM_CONTROLLER_TRANSPORT_ITRANSPORTPROXY_HPP_
