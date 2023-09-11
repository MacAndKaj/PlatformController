cmake_minimum_required(VERSION 3.18)

project(platform_controller)

set(NODE_NAME ${PROJECT_NAME}_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3")


set(SOURCE_FILES main.cpp ../src/gpio/PinController.cpp ../src/spi/SpiProxy.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} 
    PUBLIC ${CMAKE_SOURCE_DIR}/..
    PUBLIC ${CMAKE_SOURCE_DIR}/../include/platform_controller/spi
    PUBLIC ${CMAKE_SOURCE_DIR}/../include/platform_controller/gpio
)








