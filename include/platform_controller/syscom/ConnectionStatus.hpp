/**
* Copyright (c) 2026 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_CONNECTIONSTATUS_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_CONNECTIONSTATUS_HPP_


namespace platform_controller::syscom
{

enum class ConnectionStatus
{
    WAITING_FOR_CONNECTION,
    CONNECTED,
    CONNECTION_LOST,
    DISCONNECTED,
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_CONNECTIONSTATUS_HPP_
