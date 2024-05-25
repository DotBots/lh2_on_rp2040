cmake_minimum_required(VERSION 3.12)

# Set the Board
set(PICO_BOARD pico_w)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Project name
project(blink C CXX ASM)

# C and C++ standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.c
        )

# add_subdirectory(lh2)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} 
        pico_stdlib
        pico_cyw43_arch_none
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
