add_library(lh2_lib STATIC lh2.c lh2_decoder.c)

target_include_directories(lh2_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})