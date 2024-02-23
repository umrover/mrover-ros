#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1) {
    ASSERT_EQ(true, true);
}

// Declare another test
TEST(TestSuite, testCase2) {
    ASSERT_EQ(false, false);
}

// Run all the tests that were declared with TEST()
auto main(int argc, char** argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
