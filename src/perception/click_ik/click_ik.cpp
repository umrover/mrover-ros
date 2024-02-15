#include "click_ik.hpp"
#include "mrover/IK.h"

namespace mrover {

    auto ClickIkNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
        // IK Publisher
        mPcPub = mNh.advertise<IK>("/arm_ik", 1);
    }

    auto ClickIkNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        // Save points from point cloud
        points = reinterpret_cast<Point const*>(msg->data.data());
    }

    
} // namespace mrover
