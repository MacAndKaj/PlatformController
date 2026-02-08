####################################################
# Copyright (c) 2025 MacAndKaj. All rights reserved.
####################################################

set(HEADERS_DIR ${CMAKE_SOURCE_DIR}/include)

####################################################

add_library(platform_controller_roscom_if INTERFACE)

set(ROSCOM_IF_FILES
    ${HEADERS_DIR}/platform_controller/roscom/IPublisherBuilder.hpp
    ${HEADERS_DIR}/platform_controller/roscom/IRosCom.hpp
    ${HEADERS_DIR}/platform_controller/roscom/IRosComSender.hpp
)

target_sources(platform_controller_roscom_if 
    INTERFACE ${ROSCOM_IF_FILES}
)

target_include_directories(platform_controller_roscom_if 
    INTERFACE ${HEADERS_DIR}
    INTERFACE ${motoros_interfaces_INCLUDE_DIRS}
)

####################################################

set(LIBRARY_NAME platform_controller_roscom)

file(GLOB SOURCES
    # HEADERS
    ${HEADERS_DIR}/platform_controller/roscom/PublisherBuilder.hpp
    ${HEADERS_DIR}/platform_controller/roscom/PubMsg.hpp
    ${HEADERS_DIR}/platform_controller/roscom/RosCom.hpp
    ${HEADERS_DIR}/platform_controller/roscom/RosComSender.hpp
    ${HEADERS_DIR}/platform_controller/roscom/RosComSender.tpl.hpp
    # SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/RosCom.cpp
)

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME}
    # INTERNAL INCLUDES
    PUBLIC ${HEADERS_DIR}
    
    # EXTERNAL INCLUDES
    PUBLIC ${rclcpp_INCLUDE_DIRS}
    PUBLIC ${motoros_interfaces_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
    # INTERFACE LIBRARIES
    PUBLIC platform_controller_roscom_if
    # EXTERNAL LIBRARIES
    PUBLIC ${rclcpp_LIBRARIES}
    PUBLIC ${motoros_interfaces_LIBRARIES}
)
