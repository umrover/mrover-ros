#include <gtest/gtest.h>

#include <se3/se3.hpp>

TEST(Se3Test, Init) {
    auto p1 = SE3();
    ASSERT_TRUE(p1.isApprox(p1));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
//    ros::init(argc, argv, "tester");
//    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
