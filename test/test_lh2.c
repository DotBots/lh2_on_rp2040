#include <stdint.h>
#include <stdbool.h>

#include "unity.h"
#include "lh2_decoder.h"
#include "lh2_lfsr_search_test_vectors.h"

//=========================== defines =========================================

//=========================== variables ========================================

_lfsr_checkpoint_t checkpoint = { 0 };

//=========================== prototypes =======================================

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

//=========================== tests ===========================================

/**
 * @brief Test a handful of hand-picked lfsr 17-bit checkpoints, and their respective position in the sequence.
 *          from polynomial 0
 *
 */
void test_lfsr_search_poly_0(void) {

    // Select which polynomial to test
    uint8_t poly = 0;

    // test every hand picked checkpoint
    for (size_t i = 0; i < NUM_LSFR_TEST_VECTORS; i++) {
        // Perform the LFSR search
        uint32_t lfsr_result = _lfsr_index_search(&checkpoint, poly, test_lfsr_vector_table[poly][i]);
        // Compare the result of the index search to the known result in the test array
        TEST_ASSERT_EQUAL(test_lfsr_index_table[i], lfsr_result);
    }
}

/**
 * @brief Test a handful of hand-picked lfsr 17-bit checkpoints, and their respective position in the sequence.
 *          from polynomial 1
 *
 */
void test_lfsr_search_poly_1(void) {

    // Select which polynomial to test
    uint8_t poly = 1;

    // test every hand picked checkpoint
    for (size_t i = 0; i < NUM_LSFR_TEST_VECTORS; i++) {
        // Perform the LFSR search
        uint32_t lfsr_result = _lfsr_index_search(&checkpoint, poly, test_lfsr_vector_table[poly][i]);
        // Compare the result of the index search to the known result in the test array
        TEST_ASSERT_EQUAL(test_lfsr_index_table[i], lfsr_result);
    }
}

//=========================== main ===========================================

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_lfsr_search_poly_1);
    RUN_TEST(test_lfsr_search_poly_0);
    return UNITY_END();
}
