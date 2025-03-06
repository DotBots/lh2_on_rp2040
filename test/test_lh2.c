
#include "unity.h"
// #include "lh2.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}


void test_lfsr_search_poly_0(void) {
    TEST_ASSERT_EQUAL(1,2);
}


int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_lfsr_search_poly_0);
    return UNITY_END();
}


