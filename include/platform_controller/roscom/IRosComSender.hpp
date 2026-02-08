/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_ROSCOM_IROSCOMSENDER_HPP_
#define PLATFORM_CONTROLLER_ROSCOM_IROSCOMSENDER_HPP_

#include <platform_controller/roscom/PubMsg.hpp>

#include <rclcpp/rclcpp.hpp>

namespace platform_controller::roscom
{

class IRosComSender
{
public:
   virtual ~IRosComSender() = default;
   virtual void send(const PubMsg& msg) = 0;
};

} // namespace platform_controller::roscom

#endif // PLATFORM_CONTROLLER_ROSCOM_IROSCOMSENDER_HPP_
 