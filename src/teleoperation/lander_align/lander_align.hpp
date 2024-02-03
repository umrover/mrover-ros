#pragma once

#include "pch.hpp"

namespace mrover {

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        auto onInit() -> void override;

        // deprecated/not needed anymore
        // auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        auto filterNormals(sensor_msgs::PointCloud2Ptr const& cloud, float threshold) -> std::vector<const Point*>;

        auto ransac(const std::vector<Point*>& points, float distanceThreshold, int minInliers, int epochs) -> Eigen::Vector3f;
    };

} // namespace mrover
