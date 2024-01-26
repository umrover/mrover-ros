#pragma once

#include "pch.hpp"

namespace mrover {

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        auto onInit() -> void override;

        // deprecated/not needed anymore
        // auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        auto filterNormals(sensor_msgs::PointCloud2Ptr const& cloud, const int threshold) -> std::vector<Point*>;

        auto ransac(const std::vector<Point*>& filteredPoints) -> Eigen::Vector3f;
    };

} // namespace mrover
