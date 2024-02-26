#include "cost_map.hpp"

namespace mrover {

    auto CostMapNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle(); // Unused for now
        mNh.param<bool>("publish_cost_map", mPublishCostMap, true);
        mNh.param<float>("resolution", mResolution, 0.5);
        mNh.param<float>("map_width", mDimension, 30);

        mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

        mGlobalGridMsg.info.resolution = mResolution;
        mGlobalGridMsg.info.width = static_cast<int>(mDimension / mResolution);
        mGlobalGridMsg.info.height = static_cast<int>(mDimension / mResolution);
        // make center of map at (0, 0)
        mGlobalGridMsg.info.origin.position.x = -1 * mDimension / 2;
        mGlobalGridMsg.info.origin.position.y = -1 * mDimension / 2;

        mGlobalGridMsg.data.resize(mGlobalGridMsg.info.width * mGlobalGridMsg.info.height, UNKNOWN_COST);
    }

} // namespace mrover