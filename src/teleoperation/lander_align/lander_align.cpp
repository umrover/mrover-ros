#include "lander_align.hpp"
#include <random>
#include <vector>

namespace mrover {

    auto LanderAlignNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
    }

    // deprecated/not needed anymore
    // auto LanderAlignNodelet::downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr {
        // TODO: make a new point cloud with half the resolution, check the zed wrapper for how to make one
        // sensor_msgs::PointCloud2 cloudSample;
        // cloudSample.header = cloud->header
    //     cloudSample.height = cloud->height / 2;
    //     cloudSample.width = cloud->width / 2;
    //     cloudSample.fields
    // }

    auto LanderAlignNodelet::filterNormals(sensor_msgs::PointCloud2Ptr const& cloud, const int threshold) -> std::vector<Point*> {
        // TODO: return a vector of Point pointers that correspond to all points in the point cloud that are not the ground
        // filter based on whether z-normal valus exceed specified threshold

        return {};
    }

    auto LanderAlignNodelet::ransac(sensor_msgs::PointCloud2Ptr const& cloud) -> Eigen::Vector3d {
        // TODO: use RANSAC to find the lander face, should be the closest, we may need to modify this to output more information, currently the output is the normal
        auto* cloudData = reinterpret_cast<Point const*>(cloud->fields.data());

        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution (0, )
    }

} // namespace mrover
