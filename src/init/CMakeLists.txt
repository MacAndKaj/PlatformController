####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
####################################################
set(LIBRARY_NAME platform_controller_init)

set(HEADERS_DIR ${CMAKE_SOURCE_DIR}/include)

file(GLOB SOURCES
    ${HEADERS_DIR}/platform_controller/init/Context.hpp
    ${HEADERS_DIR}/platform_controller/init/IContext.hpp
    ${HEADERS_DIR}/platform_controller/init/MdcLoggingNode.hpp
    ${HEADERS_DIR}/platform_controller/init/PlatformControllerNode.hpp
    ${HEADERS_DIR}/platform_controller/init/IRosCom.hpp
    ${HEADERS_DIR}/platform_controller/init/RosCom.hpp
    ${HEADERS_DIR}/platform_controller/init/UserCommunicationNode.hpp

    ${HEADERS_DIR}/platform_controller/controllers/IHandler.hpp
    ${HEADERS_DIR}/platform_controller/controllers/SetPlatformPwmValueHandler.hpp
    ${HEADERS_DIR}/platform_controller/controllers/SetPlatformSpeedHandler.hpp

    # ${HEADERS_DIR}/platform_controller/workers/LoggingWorker.hpp

    ${CMAKE_CURRENT_SOURCE_DIR}/Context.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MdcLoggingNode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PlatformControllerNode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RosCom.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UserCommunicationNode.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/controllers/SetPlatformPwmValueHandler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers/SetPlatformSpeedHandler.cpp

    # ${CMAKE_CURRENT_SOURCE_DIR}/workers/LoggingWorker.cpp
)

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME}
    # INTERNAL INCLUDES
    PUBLIC ${HEADERS_DIR}
    
    # EXTERNAL INCLUDES
    PUBLIC ${rclcpp_INCLUDE_DIRS}
    PUBLIC ${motoros_interfaces_INCLUDE_DIRS}
    PUBLIC ${Boost_INCLUDE_DIR}
)

target_link_libraries(${LIBRARY_NAME}
    # INTERNAL LIBRARIES
    PUBLIC platform_controller_gpio
    PUBLIC platform_controller_transport
    PUBLIC platform_controller_syscom

    # EXTERNAL LIBRARIES
    PUBLIC ${rclcpp_LIBRARIES}
    PUBLIC ${motoros_interfaces_LIBRARIES}
    PUBLIC ${Boost_LIBRARIES}
)
