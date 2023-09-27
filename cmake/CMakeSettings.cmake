####################################################
# Copyright (c) 2023 M. Kajdak. All rights reserved.
####################################################

add_compile_options(-Wall -Wextra -Wpedantic)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
