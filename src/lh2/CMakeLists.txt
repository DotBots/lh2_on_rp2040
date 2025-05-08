add_library(lh2_lib STATIC lh2.c lh2_decoder.c)


# Link SDK libraries needed by the PIO header
target_link_libraries(lh2_lib
    hardware_pio
    hardware_dma
)

# Convert the PIO assembler
pico_generate_pio_header(lh2_lib ${CMAKE_CURRENT_LIST_DIR}/ts4231_capture.pio)

target_include_directories(lh2_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )