####################################################
# Copyright (c) 2023 MacAndKaj. All rights reserved.
####################################################
cmake_minimum_required(VERSION 3.8)

project(platform_controller)
set(NODE_NAME ${PROJECT_NAME}_node)

include(cmake/CMakeSettings.cmake)
include(cmake/CMakeLibraries.cmake)

# find dependencies

add_subdirectory(src)

add_executable(${NODE_NAME} src/main.cpp)

target_link_libraries(${NODE_NAME}
    PUBLIC platform_controller_init
    PUBLIC platform_controller_transport
)

target_include_directories(${NODE_NAME} 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PUBLIC $<INSTALL_INTERFACE:include>
)

install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTS)
    add_subdirectory(test)
endif ()

ament_package()
