#include <stdint.h>
#include <stdbool.h>

#include "unity.h"
#include "lh2_decoder.h"
#include "lh2_lfsr_search_test_vectors.h"

//=========================== defines =========================================

// Make a template for the the hand picked test, we will copy and paste this to automatically generate the tests
#define GENERATE_HANDPICKED_TEST(n)                   \
    void test_lfsr_search_handpicked_poly_##n(void) { \
        uint8_t poly = n;                             \
        test_lfsr_search_handpicked_generic(poly);    \
    }

#define REGISTER_HANDPICKED_TEST(n) \
    RUN_TEST(test_lfsr_search_handpicked_poly_##n);

// Make a template for the full range test.
#define GENERATE_FULL_RANGE_TEST(n)             \
    void test_lfsr_search_full_poly_##n(void) { \
        uint8_t poly = n;                       \
        test_lfsr_search_full_generic(poly);    \
    }

#define REGISTER_FULL_RANGE_TEST(n) \
    RUN_TEST(test_lfsr_search_full_poly_##n);

// Repeat a template macro 32 times, one time per polynomial index
#define REPEAT_32(TEST_MACRO)                                    \
    TEST_MACRO(0) TEST_MACRO(1) TEST_MACRO(2) TEST_MACRO(3)      \
    TEST_MACRO(4) TEST_MACRO(5) TEST_MACRO(6) TEST_MACRO(7)      \
    TEST_MACRO(8) TEST_MACRO(9) TEST_MACRO(10) TEST_MACRO(11)    \
    TEST_MACRO(12) TEST_MACRO(13) TEST_MACRO(14) TEST_MACRO(15)  \
    TEST_MACRO(16) TEST_MACRO(17) TEST_MACRO(18) TEST_MACRO(19)  \
    TEST_MACRO(20) TEST_MACRO(21) TEST_MACRO(22) TEST_MACRO(23)  \
    TEST_MACRO(24) TEST_MACRO(25) TEST_MACRO(26) TEST_MACRO(27)  \
    TEST_MACRO(28) TEST_MACRO(29) TEST_MACRO(30) TEST_MACRO(31)

//=========================== variables ========================================

// the lfsr search is expecting a pointer for the dynamic checkpoints.
// We provide a fake one.
_lfsr_checkpoint_t checkpoint = { 0 };

//=========================== functions =======================================

/**
 * @brief Test a handful of hand-picked lfsr 17-bit checkpoints generated in python
 *
 * @param[in] poly:     17-bit polynomial index, from [0, 31]
 */
void test_lfsr_search_handpicked_generic(uint8_t poly) {

    // test every hand picked checkpoint
    for (size_t i = 0; i < NUM_LSFR_TEST_VECTORS; i++) {
        // Perform the LFSR search
        uint32_t lfsr_result = _lfsr_index_search(&checkpoint, poly, test_lfsr_vector_table[poly][i]);
        // Compare the result of the index search to the known result in the test array
        TEST_ASSERT_EQUAL(test_lfsr_index_table[i], lfsr_result);
    }
}

/**
 * @brief Steps an LFSR sequence one step forward
 *
 * @param[in] sequence: 17-bit lfsr sequence
 * @param[in] poly:     17-bit polynomial index, from [0, 31]
 *
 * @return    sequence+1:     Next step in the LFSR sequence
 */
uint32_t lfsr_step_forward(uint32_t sequence, uint8_t poly) {

    // LSFR forward update
    bool b1 = __builtin_popcount(sequence & test_polynomials[poly]) & 0x01;  // mask the buffer w/ the selected polynomial
    return ((sequence << 1) | b1) & (0x0001FFFF);          // Add the new bit to the sequence
}

/**
 * @brief exhaustively test every possible step in the lfsr sequence, from 0x01 all the way to the 120k step
 *        (120k is close enough to the maximum sequence lenght of 2^17)
 *
 * @param[in] poly:     17-bit polynomial index, from [0, 31]
 */
void test_lfsr_search_full_generic(uint8_t poly) {

    // Set the starting seed of the lfsr sequence
    uint32_t sequence = 0x01;

    // test every hand picked checkpoint
    for (size_t i = 0; i < 120000; i++) {
        // Perform the LFSR search
        uint32_t lfsr_result = _lfsr_index_search(&checkpoint, poly, sequence);
        // Compare the result of the lfsr search to the known result
        TEST_ASSERT_EQUAL(i, lfsr_result);

        // Step the lfsr sequence forward
        sequence = lfsr_step_forward(sequence, poly);
    }
}

//=========================== tests ===========================================

// LFSR HANDPICKED CHECKPOINT TESTS
// Generate all 32 from the template macro
REPEAT_32(GENERATE_HANDPICKED_TEST);

// LFSR FULL RANGE TESTS
// Generate all 32 from the template macro
REPEAT_32(GENERATE_FULL_RANGE_TEST);

void test_lfsr_search_should_be_FFFFFFFF_when_input_0(void) {
    // test that inputing 0, causes an error
    uint32_t lfsr_result = _lfsr_index_search(&checkpoint, 0, 0x00000000);
    TEST_ASSERT_EQUAL(LH2_LFSR_SEARCH_ERROR_INDICATOR, lfsr_result);
}

//=========================== main ===========================================

int main(void) {
    UNITY_BEGIN();
    // Register LFSR handpicked checkpoint test, using a template MACRO
    REPEAT_32(REGISTER_HANDPICKED_TEST);
    REPEAT_32(REGISTER_FULL_RANGE_TEST);
    RUN_TEST(test_lfsr_search_should_be_FFFFFFFF_when_input_0);
    // Test non
    return UNITY_END();
}

//=========================== private ===========================================

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}
