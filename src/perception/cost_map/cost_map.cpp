#include "cost_map.hpp"

namespace mrover {

    auto CostMapNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPublishCostMap = mNh.param<bool>("publish_cost_map", true);
        mResolution = mNh.param<float>("resolution", 0.5);
        mDimension = mNh.param<float>("map_width", 30);
        mWorldFrame = mNh.param<std::string>("map_frame", "map");

        mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mServer = mNh.advertiseService("move_cost_map", &CostMapNodelet::moveCostMapCallback, this);

        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

        mGlobalGridMsg.info.resolution = mResolution;
        mGlobalGridMsg.info.width = static_cast<int>(mDimension / mResolution);
        mGlobalGridMsg.info.height = static_cast<int>(mDimension / mResolution);
        // Center the map at (0, 0)
        mGlobalGridMsg.info.origin.position.x = -mDimension / 2;
        mGlobalGridMsg.info.origin.position.y = -mDimension / 2;

        mGlobalGridMsg.data.resize(mGlobalGridMsg.info.width * mGlobalGridMsg.info.height, UNKNOWN_COST);
    }

    auto CostMapNodelet::moveCostMapCallback(MoveCostMap::Request& req, MoveCostMap::Response& res) -> bool {
        SE3d waypointPos = SE3Conversions::fromTfTree(mTfBuffer, req.course, mWorldFrame);
        std::ranges::fill(mGlobalGridMsg.data, UNKNOWN_COST);
        mGlobalGridMsg.info.origin.position.x = waypointPos.x() - mDimension / 2;
        mGlobalGridMsg.info.origin.position.y = waypointPos.y() - mDimension / 2;
        res.success = true;
        return true;
    }
} // namespace mrover