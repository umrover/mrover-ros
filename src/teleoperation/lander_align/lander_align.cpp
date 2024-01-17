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

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "lander_align");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/LanderAlignNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LanderAlignNodelet, nodelet::Nodelet)
#endif
