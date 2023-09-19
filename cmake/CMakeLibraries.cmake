####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
####################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(motoros_interfaces REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
endif()
