####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
####################################################
set(LIBRARY_NAME platform_controller_transport)

set(HEADERS_DIR ${CMAKE_SOURCE_DIR}/include)

file(GLOB SOURCES
    ${HEADERS_DIR}/platform_controller/transport/ITransportProxy.hpp
    ${HEADERS_DIR}/platform_controller/transport/SerialProxy.hpp
    ${HEADERS_DIR}/platform_controller/transport/SpiProxy.hpp

    ${CMAKE_CURRENT_SOURCE_DIR}/SerialProxy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SpiProxy.cpp
)

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME}
    PUBLIC ${HEADERS_DIR}
    PUBLIC ${rclcpp_INCLUDE_DIRS}
    PUBLIC ${motoros_interfaces_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
    PUBLIC ${rclcpp_LIBRARIES}
    PUBLIC ${motoros_interfaces_LIBRARIES}
)
