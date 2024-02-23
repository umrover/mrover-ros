#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include "click_ik.hpp"
#include "mrover/ClickIkAction.h"
#include "mrover/ClickIkGoal.h"
#include "mrover/IK.h"

namespace mrover {

    void ClickIkNodelet::execute(const mrover::ClickIkGoalConstPtr& goal) {
        ROS_INFO("Executing goal");
    }
    
    void ClickIkNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
        // IK Publisher
        mIkPub = mNh.advertise<IK>("/arm_ik", 1);

        server.start();

    }

    void ClickIkNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        // Save points from point cloud
        mPoints = reinterpret_cast<Point const*>(msg->data.data());
        mNumPoints = msg->width * msg->height;
    }


    
} // namespace mrover
