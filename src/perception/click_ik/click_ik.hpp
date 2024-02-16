#pragma once

#include "pch.hpp"
#include <actionlib/server/simple_action_server.h>

namespace mrover {

    class ClickIkNodelet final : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        ros::Subscriber mPcSub;

        // IK publisher
        ros::Publisher mIkPub;

        const Point* mPoints{};

        std::size_t mNumPoints{};
        
    public:
        ClickIkNodelet() = default;

        ~ClickIkNodelet() override = default;

        void onInit() override;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        void executeGoal();
    };

} // namespace mrover
