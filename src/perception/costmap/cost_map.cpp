#include "cost_map.hpp"

namespace mrover {

    void CostMapNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mNh.param<bool>("publish_cost_maps", publishCostMaps, true);
        mNh.param<bool>("verbose", verbose, false);
        mNh.param<float>("element_size", element_size, 1);
        mNh.param<int>("map_width", map_width, 1);
        mNh.param<int>("map_height", map_height, 1);
        mNh.param<float>("cutoff", cutoff, 1);
        

        //mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("cost_maps", 1);

        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);

    }
}

/*
TODO:
- Nodelet takes in pointcloud
PARALLELIZE
- Get cost of point
- Transform point to nav basis
- Stitch to nav grid
END

*/