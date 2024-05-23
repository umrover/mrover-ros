#include "cost_map.hpp"

namespace mrover {

    auto CostMapNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mResolution = mNh.param<float>("resolution", 0.5);
        mSize = mNh.param<float>("size", 30);
        mMapFrame = mNh.param<std::string>("map_frame", "map");
        mNearClip = mNh.param<float>("near_clip", 1);
        mFarClip = mNh.param<float>("far_clip", 3);

        mCostMapPub = mNh.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mServer = mNh.advertiseService("move_cost_map", &CostMapNodelet::moveCostMapCallback, this);

        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

        mGlobalGridMsg.info.resolution = mResolution;
        mGlobalGridMsg.info.width = static_cast<int>(mSize / mResolution);
        mGlobalGridMsg.info.height = static_cast<int>(mSize / mResolution);
        // Center the map at (0, 0)
        mGlobalGridMsg.header.frame_id = mMapFrame;
        mGlobalGridMsg.info.origin.position.x = -mSize / 2;
        mGlobalGridMsg.info.origin.position.y = -mSize / 2;

        mGlobalGridMsg.data.resize(mGlobalGridMsg.info.width * mGlobalGridMsg.info.height, UNKNOWN_COST);
    }

} // namespace mrover