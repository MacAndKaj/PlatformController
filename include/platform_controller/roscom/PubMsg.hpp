/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_PUBMSG_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_PUBMSG_HPP_

#include <motoros_interfaces/msg/generic_status.hpp>
#include <motoros_interfaces/msg/platform_status.hpp>

#include <variant>

namespace platform_controller::roscom
{

namespace mi_messages = motoros_interfaces::msg;

using PubMsg = std::variant<
    mi_messages::GenericStatus, 
    mi_messages::PlatformStatus
    >;

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_IROSCOMSENDER_HPP_
