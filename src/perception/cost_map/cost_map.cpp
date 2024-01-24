#include "cost_map.hpp"

namespace mrover {

    void CostMapNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle(); // Unused for now
        mNh.param<bool>("publish_cost_map", mPublishCostMap, true);
        mNh.param<float>("resolution", mResolution, 1);
        mNh.param<float>("map_width", mDimension, 30);

        mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        //TODO: Change topic to whatever the occupancy grid creating node publishes to
        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

        mGlobalGridMsg.info.resolution = mResolution;
        mGlobalGridMsg.info.width = (int) (mDimension / mResolution);
        mGlobalGridMsg.info.height = (int) (mDimension / mResolution);
        //TODO: What do we choose as our original load time and origin?
        //globalGridMsg.info.map_load_time =
        //globalGridMsg.info.origin =

        //TODO: Why is int8 not recognized?
        mGlobalGridMsg.data.resize(mGlobalGridMsg.info.width * mGlobalGridMsg.info.height);
        std::fill(mGlobalGridMsg.data.begin(), mGlobalGridMsg.data.end(), -1);
    }
} // namespace mrover

/*
TODO:
- Nodelet takes in pointcloud
PARALLELIZE
- Get cost of point
- Transform point to nav basis
- Stitch to nav grid
END

*/
