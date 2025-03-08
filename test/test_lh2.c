#include <stdint.h>
#include <stdbool.h>

#include "unity.h"
#include "lh2_decoder.h"
#include "lh2_lfsr_search_test_vectors.h"

//=========================== defines =========================================

// Make a template for the the hand picked test, we will copy and paste this to automatically generate the tests
#define GENERATE_HANDPICKED_TEST(n) \
    void test_lfsr_search_handpicked_poly_##n(void) { \
        uint8_t poly = n; \
        test_lfsr_search_handpicked_generic(poly); \
    }

#define REGISTER_HANDPICKED_TEST(n) \
    RUN_TEST(test_lfsr_search_handpicked_poly_##n);

// Repeat a template macro 32 times, one time per polynomial index
#define REPEAT_32(TEST_MACRO) \
    TEST_MACRO(0)  TEST_MACRO(1)  TEST_MACRO(2)  TEST_MACRO(3)  \
    TEST_MACRO(4)  TEST_MACRO(5)  TEST_MACRO(6)  TEST_MACRO(7)  \
    TEST_MACRO(8)  TEST_MACRO(9)  TEST_MACRO(10) TEST_MACRO(11) \
    TEST_MACRO(12) TEST_MACRO(13) TEST_MACRO(14) TEST_MACRO(15) \
    TEST_MACRO(16) TEST_MACRO(17) TEST_MACRO(18) TEST_MACRO(19) \
    TEST_MACRO(20) TEST_MACRO(21) TEST_MACRO(22) TEST_MACRO(23) \
    TEST_MACRO(24) TEST_MACRO(25) TEST_MACRO(26) TEST_MACRO(27) \
    TEST_MACRO(28) TEST_MACRO(29) TEST_MACRO(30) TEST_MACRO(31)

//=========================== variables ========================================

// the lfsr search is expecting a pointer for the dynamic checkpoints.
// We provide a fake one.
_lfsr_checkpoint_t checkpoint = { 0 };

//=========================== prototypes =======================================

/**
 * @brief Test a handful of hand-picked lfsr 17-bit checkpoints generated in python, for polynomial 0
 *
 * @param[in] poly:     17-bit polynomial, from [0, 31]
 */
void test_lfsr_search_handpicked_generic(uint8_t poly);

//=========================== tests ===========================================

// LFSR HANDPICKED CHECKPOINT TESTS
// Generate all 32 from the template macro
REPEAT_32(GENERATE_HANDPICKED_TEST) 


//=========================== main ===========================================

int main(void) {
    UNITY_BEGIN();
    // Register LFSR handpicked checkpoint test, using a template MACRO
    REPEAT_32(REGISTER_HANDPICKED_TEST) 
    return UNITY_END();
}

//=========================== private ===========================================

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_lfsr_search_handpicked_generic(uint8_t poly) {

    // test every hand picked checkpoint
    for (size_t i = 0; i < NUM_LSFR_TEST_VECTORS; i++) {
        // Perform the LFSR search
        uint32_t lfsr_result = _lfsr_index_search(&checkpoint, poly, test_lfsr_vector_table[poly][i]);
        // Compare the result of the index search to the known result in the test array
        TEST_ASSERT_EQUAL(test_lfsr_index_table[i], lfsr_result);
    }
}
