cmake_minimum_required(VERSION 3.12)
project(UnitTests C CXX ASM)

# C and C++ standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

enable_testing()

# Add Unity test framework
add_library(Unity STATIC unity/unity.c)
target_include_directories(Unity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/unity)


# Create the test executables LFSR Index Search
add_executable(test_lh2_lfsr_index_search 
        test_lh2_lfsr_index_search.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/lh2/lh2_decoder.c)
target_link_libraries(test_lh2_lfsr_index_search Unity)
target_include_directories(test_lh2_lfsr_index_search PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src/lh2)

# Create the test executables LFSR Determine Polynomial
# add_executable(test_lh2_determine_polynomial 
#         test_lh2_determine_polynomial.c
#         ${CMAKE_CURRENT_SOURCE_DIR}/../src/lh2/lh2_decoder.c)
# target_link_libraries(test_lh2_determine_polynomial Unity)
# target_include_directories(test_lh2_determine_polynomial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src/lh2)


# Add Tests
add_test(NAME TestLH2_LFSR_search COMMAND test_lh2_lfsr_index_search)
# add_test(NAME TestLH2_Determine_Poly COMMAND test_lh2_determine_polynomial)


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
