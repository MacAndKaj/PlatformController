####################################################
# Copyright (c) 2025 MacAndKaj. All rights reserved.
####################################################
set(MOCK_DIR ${CMAKE_SOURCE_DIR}/test/ut/mocks)

project(test_platform_controller_init_services)

ament_add_gmock(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/PlatformStatusPollingServiceTests.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${MOCK_DIR}

    # EXTERNAL INCLUDES
    PUBLIC ${rclcpp_INCLUDE_DIRS}
    PUBLIC ${motoros_interfaces_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    platform_controller_init
    platform_controller_syscom_defs
    platform_controller_syscom
    
    ${rclcpp_LIBRARIES}
    ${motoros_interfaces_LIBRARIES}
)
