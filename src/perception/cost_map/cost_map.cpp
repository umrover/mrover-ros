#include "cost_map.hpp"

namespace mrover {

    void CostMapNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle(); // Unused for now
        mNh.param<bool>("publish_cost_map", mPublishCostMap, true);
        mNh.param<float>("resolution", mResolution, 2);
        mNh.param<int>("map_width", mDimension, 15);

        mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        //TODO: Change topic to whatever the occupancy grid creating node publishes to
        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

        mGlobalGridMsg.info.resolution = mResolution;
        mGlobalGridMsg.info.width = mDimension;
        mGlobalGridMsg.info.height = mDimension;
        //TODO: What do we choose as our original load time and origin?
        //globalGridMsg.info.map_load_time =
        //globalGridMsg.info.origin =

        //TODO: Why is int8 not recognized?
        std::vector<signed char> initial_map;
        std::fill(initial_map.begin(), initial_map.end(), -1);
        mGlobalGridMsg.data = initial_map;
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
