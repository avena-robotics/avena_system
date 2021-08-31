#include <gtest/gtest.h>

#include "helpers_commons/geometry.hpp"

TEST(getAxesDiffBetweenQuaternions, testOrientationEqual)
{
    // Initialization
    Eigen::Quaternionf quat0(1, 0, 0, 0);
    // 0 degrees rotation around x, y, z axes
    Eigen::Quaternionf quat1 = Eigen::AngleAxisf(0, Eigen::Vector3f(1, 0, 0)) * Eigen::AngleAxisf(0, Eigen::Vector3f(0, 1, 0)) * Eigen::AngleAxisf(0, Eigen::Vector3f(0, 0, 1));
    
    auto quat_diff = helpers::commons::getAxesDiffBetweenQuaternions(quat0, quat1);

    auto desired_angle = 0.0;
    EXPECT_DOUBLE_EQ(quat_diff[0], desired_angle);
    EXPECT_DOUBLE_EQ(quat_diff[1], desired_angle);
    EXPECT_DOUBLE_EQ(quat_diff[2], desired_angle);
}

TEST(getDiffBetweenAffines, testDistanceEqual)
{
    // Initialization
    Eigen::Translation3d pos0(0, 0, 0);
    Eigen::Quaterniond quat0(1, 0, 0, 0);
    Eigen::Affine3d aff0 = pos0 * quat0;

    Eigen::Translation3d pos1(0, 0, 0);
    Eigen::Quaterniond quat1 = Eigen::AngleAxisd(1, Eigen::Vector3d(1, 0, 0)) * Eigen::AngleAxisd(-1.7, Eigen::Vector3d(0, 1, 0)) * Eigen::AngleAxisd(0.5, Eigen::Vector3d(0, 0, 1));
    Eigen::Affine3d aff1 = pos1 * quat1;
    
    auto aff_diff = helpers::commons::getDiffBetweenAffines(aff0, aff1);
    
    auto desired_distance = 0.0;
    EXPECT_DOUBLE_EQ(aff_diff[0], desired_distance);
}

TEST(getDiffBetweenAffines, testOrientationEqual)
{
    // Initialization
    Eigen::Translation3d pos0(0, 0, 0);
    Eigen::Quaterniond quat0(1, 0, 0, 0);
    Eigen::Affine3d aff0 = pos0 * quat0;

    Eigen::Translation3d pos1(0, 1, 0);
    // 0 degrees rotation around x, y, z axes
    Eigen::Quaterniond quat1 = Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0)) * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0)) * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
    Eigen::Affine3d aff1 = pos1 * quat1;
    
    auto aff_diff = helpers::commons::getDiffBetweenAffines(aff0, aff1);

    auto desired_angle = 0.0;
    EXPECT_DOUBLE_EQ(aff_diff[1], desired_angle);
    EXPECT_DOUBLE_EQ(aff_diff[2], desired_angle);
    EXPECT_DOUBLE_EQ(aff_diff[3], desired_angle);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    bool all_successful = RUN_ALL_TESTS();

    return all_successful;
}
