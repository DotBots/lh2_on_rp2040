cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Project name
project(lh2 C CXX ASM)

# C and C++ standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(${PROJECT_NAME}
        src/main.c
        # src/lh2/lh2.c
        )

# Add lh2 files
add_subdirectory(src/lh2)
target_link_libraries(${PROJECT_NAME} lh2_lib)
# include_directories(${CMAKE_SOURCE_DIR}/src/lh2)
        
# Conver the PIO assembler
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/ts4231_capture.pio)

# Make a list of common dependencies
set(COMMON_LIBS pico_stdlib hardware_pio hardware_dma pico_multicore)

# Add board-specific dependencies
if (PICO_BOARD STREQUAL "pico_w" OR BOARD STREQUAL "pico2_w")
    list(APPEND COMMON_LIBS pico_cyw43_arch_none)
endif()

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS})

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

# enable USB CDC serial
pico_enable_stdio_usb(lh2 1)
pico_enable_stdio_uart(lh2 0)