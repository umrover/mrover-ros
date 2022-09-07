#include <gtest/gtest.h>

#include <se3/se3.hpp>

TEST(Se3Test, Init) {
    SE3 p1;
    ASSERT_TRUE(p1.posVector().isApprox(SE3::Vector{}));
    ASSERT_TRUE(p1.rotation().rotationQuaternion().isApprox(SO3::Quaternion::Identity()));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
