#include <actionlib/server/simple_action_server.h>
#include "click_ik.hpp"
#include "mrover/IK.h"

namespace mrover {

    void ClickIkNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
        // IK Publisher
        mIkPub = mNh.advertise<IK>("/arm_ik", 1);

        // Start ActionServer
        actionlib::SimpleActionServer<mrover::ClickIkAction> server(mNh, "do_click_ik", [&](const mrover::ClickIkGoalConstPtr& goal) {
            // How to pass server to callback? It will go out of scope and can't be assigned to a member variable because actionlib::SimpleActionServer is non-copyable
        }, false);
        server.start();

    }

    void ClickIkNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        // Save points from point cloud
        mPoints = reinterpret_cast<Point const*>(msg->data.data());
        mNumPoints = msg->width * msg->height;
    }

    
} // namespace mrover
