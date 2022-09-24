#include <gtest/gtest.h>

#include <se3/se3.hpp>

constexpr double TOLERANCE = 1e-3;

TEST(Se3Test, DefaultInitialization) {
    SE3 p;
    ASSERT_TRUE(p.posVector().isApprox(SE3::Vector{}));
    ASSERT_TRUE(p.rotation().rotationQuaternion().isApprox(SO3::Quaternion::Identity()));
}

TEST(SE3Test, Consutrctor) {
    SO3::Quaternion q{1, 2, 3, 4};
    SE3::Vector v{1, 2, 3};
    SE3 p{v, SO3{q}};
    ASSERT_TRUE(p.posVector().isApprox(v));
    ASSERT_FALSE(p.rotation().rotationQuaternion().isApprox(q));
    ASSERT_TRUE(p.rotation().rotationQuaternion().isApprox(q.normalized()));
}

TEST(SE3Test, DistanceTwoNonzero) {
    SE3 p1({1, 2, 3}, SO3::identity());
    SE3 p2({-4, 8, -7}, SO3::identity());
    ASSERT_NEAR(p1.posDistanceTo(p2), 12.6886, TOLERANCE);
    ASSERT_NEAR(p2.posDistanceTo(p1), 12.6886, TOLERANCE);
    ASSERT_NEAR(p1.posDistanceTo(p1), 0.0, TOLERANCE);
    ASSERT_NEAR(p2.posDistanceTo(p2), 0.0, TOLERANCE);
}

TEST(SE3Test, DistanceOneZeroOneNonzero) {
    SE3 p1;
    SE3 p2({3, 4, 0}, SO3::identity());
    ASSERT_NEAR(p1.posDistanceTo(p2), 5.0, TOLERANCE);
    ASSERT_NEAR(p2.posDistanceTo(p1), 5.0, TOLERANCE);
}

TEST(SE3Test, DistanceTwoZero) {
    SE3 p1, p2;
    ASSERT_NEAR(p1.posDistanceTo(p2), 0.0, TOLERANCE);
    ASSERT_NEAR(p2.posDistanceTo(p1), 0.0, TOLERANCE);
}

TEST(SE3Test, IsApproxIdentity) {
    SE3 p1, p2;
    ASSERT_TRUE(p1.isApprox(p1));
    ASSERT_TRUE(p2.isApprox(p2));
    ASSERT_TRUE(p1.isApprox(p2));
    ASSERT_TRUE(p2.isApprox(p1));
}

TEST(SE3Test, IsApproxDifferentRotation) {
    SE3 p1({1, 2, 3}, SO3{{1, 2, 3, 4}});
    SE3 p2({1, 2, 3}, SO3{{4, 3, 2, 1}});
    ASSERT_FALSE(p1.isApprox(p2));
}

TEST(SE3Test, IsApproxDifferentPosition) {
    SE3 p1({1, 2, 3}, SO3{{1, 2, 3, 4}});
    SE3 p2({3, 2, 1}, SO3{{1, 2, 3, 4}});
    ASSERT_FALSE(p1.isApprox(p2));
}

TEST(SE3Test, IsApproxJustOverTolerance) {
    SE3 p1({1, 2, 3}, SO3{{1, 2, 3, 4}});
    SE3 p2({1.004, 2, 3}, SO3{{1.0005, 2, 3, 4}});
    ASSERT_FALSE(p1.isApprox(p2, 1e-3));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
