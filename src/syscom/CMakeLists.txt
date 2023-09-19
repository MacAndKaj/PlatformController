####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
####################################################

set(HEADERS_DIR ${CMAKE_SOURCE_DIR}/include)

####################################################

add_library(platform_controller_syscom_defs INTERFACE)

set(DEFS_FILES
    ${HEADERS_DIR}/platform_controller/syscom/defs/Frame.hpp
    ${HEADERS_DIR}/platform_controller/syscom/defs/Messages.hpp
)

target_sources(platform_controller_syscom_defs 
    INTERFACE ${DEFS_FILES}
)

target_include_directories(platform_controller_syscom_defs 
    INTERFACE ${HEADERS_DIR}
)

####################################################

project(platform_controller_syscom)

file(GLOB SOURCES
    # HEADERS
    ${HEADERS_DIR}/platform_controller/syscom/ISysCom.hpp
    ${HEADERS_DIR}/platform_controller/syscom/SysCom.hpp

    # SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/SysCom.cpp
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
    # INTERNAL LIBRARIES
    PUBLIC platform_controller_transport
    PUBLIC platform_controller_syscom_defs

    # EXTERNAL LIBRARIES
    PUBLIC ${rclcpp_LIBRARIES}
    PUBLIC ${motoros_interfaces_LIBRARIES}
)
