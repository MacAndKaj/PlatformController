/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_TRANSPORT_TRANSPORTPROXYMOCK_HPP_
#define PLATFORM_CONTROLLER_TRANSPORT_TRANSPORTPROXYMOCK_HPP_

#include <platform_controller/transport/ITransportProxy.hpp>

#include <cstdint>
#include <vector>

namespace platform_controller::transport
{

class TransportProxyMock : public ITransportProxy
{
public:
    MOCK_METHOD(bool, send, (const std::vector<std::uint8_t>&));
    MOCK_METHOD(std::vector<std::uint8_t>, read, ());
    MOCK_METHOD(std::vector<std::uint8_t>, read, (unsigned int nbytes));
};

} // namespace platform_controller::transport

#endif // PLATFORM_CONTROLLER_TRANSPORT_TRANSPORTPROXYMOCK_HPP_
