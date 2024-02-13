#include "click_ik.hpp"

namespace mrover {

    auto ClickIkNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
    }

    auto ClickIkNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
    }

} // namespace mrover
