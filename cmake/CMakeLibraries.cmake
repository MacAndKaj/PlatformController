####################################################
# Copyright (c) 2023 MacAndKaj. All rights reserved.
####################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
message(STATUS "rclcpp include path found: ${rclcpp_INCLUDE_DIRS}")

find_package(Boost 1.86 COMPONENTS json)

find_package(motoros_interfaces REQUIRED)

message(STATUS "motoros_interfaces include path found: ${motoros_interfaces_INCLUDE_DIRS}")
message(STATUS "motoros_interfaces libraries found: ${motoros_interfaces_LIBRARIES}")

if(BUILD_TESTS)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
endif()
