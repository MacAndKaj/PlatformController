####################################################
# Copyright (c) 2023 MacAndKaj. All rights reserved.
####################################################

set(HEADERS_DIR ${CMAKE_SOURCE_DIR}/include)

project(platform_controller_gpio)

file(GLOB SOURCES
    # HEADERS
    ${HEADERS_DIR}/platform_controller/gpio/defs.hpp
    ${HEADERS_DIR}/platform_controller/gpio/IGpioManager.hpp
    ${HEADERS_DIR}/platform_controller/gpio/GpioManager.hpp
    ${HEADERS_DIR}/platform_controller/gpio/Rpi3BPlusGpioChips.hpp

    # SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GpioManager.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME}
    # INTERNAL INCLUDES
    PUBLIC ${HEADERS_DIR}
    
    # EXTERNAL INCLUDES
    PUBLIC ${rclcpp_INCLUDE_DIRS}
    PUBLIC ${motoros_interfaces_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    # EXTERNAL LIBRARIES
    PUBLIC ${rclcpp_LIBRARIES}
    PUBLIC ${motoros_interfaces_LIBRARIES}
)
