#pragma once

#include "pch.hpp"

namespace mrover {

    class ClickIkNodelet final : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        ros::Subscriber mPcSub;

        // IK publisher
        ros::Publisher mPcPub;

        const Point* points{};
        
    public:
        ClickIkNodelet() = default;

        ~ClickIkNodelet() override = default;

        void onInit() override;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        void executeGoal(auto g);
    };

} // namespace mrover
