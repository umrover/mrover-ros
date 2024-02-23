#pragma once

#include "mrover/ClickIkAction.h"
#include "pch.hpp"
#include <actionlib/server/simple_action_server.h>

namespace mrover {

    class ClickIkNodelet final : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        ros::Subscriber mPcSub;

        // IK publisher
        ros::Publisher mIkPub;

        actionlib::SimpleActionServer<mrover::ClickIkAction> server = actionlib::SimpleActionServer<mrover::ClickIkAction>(mNh, "do_click_ik", [&](const mrover::ClickIkGoalConstPtr& goal) {
            execute(goal);
        }, false);
        const Point* mPoints{};

        std::size_t mNumPoints{};
        
    public:
        ClickIkNodelet() = default;

        ~ClickIkNodelet() override = default;

        void onInit() override;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        void execute(const mrover::ClickIkGoalConstPtr& goal);
    };

} // namespace mrover
