#include "lander_align.hpp"

namespace mrover {

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
    }

    auto LanderAlignNodelet::downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr {
        // TODO: make a new point cloud with half the resolution, check the zed wrapper for how to make one
    }

    auto LanderAlignNodelet::filterNormals(sensor_msgs::PointCloud2Ptr const& cloud) -> void {
        // TODO: modify "cloud" in-place, this should be input from the downsample function
    }

    auto LanderAlignNodelet::ransac(sensor_msgs::PointCloud2Ptr const& cloud) -> Eigen::Vector3d {
        // TODO: use RANSAC to find the lander face, should be the closest, we may need to modify this to output more information, currently the output is the normal
    }

} // namespace mrover
