#pragma once

#include "pch.hpp"

namespace mrover {

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mDebugVectorPub;

        ros::Subscriber mVectorSub;


        float mThreshold;

        std::vector<Point const*> mFilteredPoints;

        auto onInit() -> void override;

        // deprecated/not needed anymore
        // auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        void LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud);

        void filterNormals(sensor_msgs::PointCloud2Ptr const& cloud);

        auto ransac(std::vector<Point const*> const& points, float distanceThreshold, int minInliers, int epochs) -> Eigen::Vector3f;
    };

} // namespace mrover