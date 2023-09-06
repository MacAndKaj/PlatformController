####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
####################################################
set(LIBRARY_NAME platform_controller_init)

set(HEADERS_DIR ${CMAKE_SOURCE_DIR}/include)

file(GLOB SOURCES
    ${HEADERS_DIR}/platform_controller/init/Context.hpp
    ${HEADERS_DIR}/platform_controller/init/IContext.hpp
    ${HEADERS_DIR}/platform_controller/init/PlatformControllerNode.hpp

    ${CMAKE_CURRENT_SOURCE_DIR}/Context.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PlatformControllerNode.cpp
)

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME}
    # INTERNAL INCLUDES
    PUBLIC ${HEADERS_DIR}

    # EXTERNAL INCLUDES
    PUBLIC ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
    # INTERNAL LIBRARIES
    PUBLIC platform_controller_transport

    # EXTERNAL LIBRARIES
    PUBLIC ${rclcpp_LIBRARIES}
)
