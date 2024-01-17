#pragma once

#include "pch.hpp"

namespace mrover {

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        auto onInit() -> void override;

        auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        auto filterNormals(sensor_msgs::PointCloud2Ptr const& cloud) -> void;

        auto ransac(sensor_msgs::PointCloud2Ptr const& cloud) -> Eigen::Vector3d;
    };

} // namespace mrover
