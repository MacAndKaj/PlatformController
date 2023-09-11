####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
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

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
