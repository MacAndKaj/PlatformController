/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_

#include <platform_controller/syscom/Request.hpp>
#include <platform_controller/syscom/Response.hpp>

#include <cstdint>
#include <functional>

namespace platform_controller::syscom
{

using MessageId = std::uint8_t;
using Callback = std::function<void(const Response)>;

class ISysCom
{
public:
    virtual ~ISysCom() = default;
    /**
     * @brief Main loop of the SysCom
     * @details This function should be called in a loop to process incoming messages
     */
    virtual void work() = 0;
    /**
     * @brief Send message to the platform
     * @param msg Message to send
     * @return true if message was sent successfully, false otherwise
     */
    virtual bool send(const Request& msg) = 0;
    /**
     * @brief Subscribe to a message
     * @param msgId Message ID to subscribe to
     * @param callback Callback function to be called when message is received
     * @return Subscription ID
     */
    virtual int subscribe(MessageId msgId, const Callback& callback) = 0;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_
