#include "cost_map.hpp"

namespace mrover {

    auto CostMapNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle(); // Unused for now
        mNh.param<bool>("publish_cost_map", mPublishCostMap, true);
        mNh.param<float>("resolution", mResolution, 0.5);
        mNh.param<float>("map_width", mDimension, 30);

        mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mServer = mNh.advertiseService("move_cost_map", &CostMapNodelet::moveCostMapCallback, this);

        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

        mGlobalGridMsg.info.resolution = mResolution;
        mGlobalGridMsg.info.width = static_cast<int>(mDimension / mResolution);
        mGlobalGridMsg.info.height = static_cast<int>(mDimension / mResolution);
        // make center of map at (0, 0)
        mGlobalGridMsg.info.origin.position.x = -1 * mDimension / 2;
        mGlobalGridMsg.info.origin.position.y = -1 * mDimension / 2;

        mGlobalGridMsg.data.resize(mGlobalGridMsg.info.width * mGlobalGridMsg.info.height, UNKNOWN_COST);
    }

    auto CostMapNodelet::moveCostMapCallback(MoveCostMap::Request& req, MoveCostMap::Response& res) -> bool {
        SE3d waypointPos = SE3Conversions::fromTfTree(mTfBuffer, req.course, "map");
        mGlobalGridMsg.info.origin.position.x = waypointPos.x() - 1 * mDimension / 2;
        mGlobalGridMsg.info.origin.position.y = waypointPos.y() - 1 * mDimension / 2;
        res.success = true;
        return true;
    }
} // namespace mrover