/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_

#include <platform_controller/syscom/Command.hpp>

#include <cstdint>
#include <functional>

namespace platform_controller::syscom
{

using MessageId = std::uint8_t;
using Callback = std::function<void(const PlatformStatus)>;

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
     * @param cmd Message to send
     */
    virtual void send(const Command& cmd) = 0;
    /**
     * @brief Subscribe to a message PlatformStatus
     * @param callback Callback function to be called when message is received
     * @return Subscription ID
     */
    virtual int subscribeForStatus(const Callback& callback) = 0;

    /**
     *  @brief Set syscom debug mode - ignores some communication errors and prints more logs
     * @param enabled boolean value indicating whether debug mode should be enabled or not
     */
    virtual void setDebug(bool enabled) = 0;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_ISYSCOM_HPP_
