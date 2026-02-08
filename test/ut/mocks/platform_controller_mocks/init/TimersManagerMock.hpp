/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
 #ifndef PLATFORM_CONTROLLER_INIT_TIMERSMANAGERMOCK_HPP_
 #define PLATFORM_CONTROLLER_INIT_TIMERSMANAGERMOCK_HPP_
 
 #include <platform_controller/init/ITimersManager.hpp>
 
 #include <rclcpp/rclcpp.hpp>
 
 #include <memory>
 
 namespace platform_controller::init
 {
 
 class TimersManagerMock : public ITimersManager
 {
 public:
    MOCK_METHOD(int,
        startCyclicTimer,
        (std::chrono::milliseconds period, const std::function<void()>&));
 };
 
 } // namespace platform_controller::init
 
 #endif // PLATFORM_CONTROLLER_INIT_TIMERSMANAGERMOCK_HPP_
 