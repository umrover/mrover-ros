#include "cost_map.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>

namespace mrover {

    void CostMapNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle(); // Unused for now
        mNh.param<bool>("publish_cost_maps", publishCostMaps, true);
        mNh.param<bool>("verbose", verbose, false);
        mNh.param<float>("resolution", resolution, 1);
        mNh.param<int>("map_width", map_width, 1);
        mNh.param<int>("map_height", map_height, 1);


        mCostMapPub = mCmt.advertise<nav_msgs::OccupancyGrid>("cost_maps", 1); // We publish our results to "cost_maps"

        //TODO: Change topic to whatever the occupancy grid creating node publishes to
        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::occupancyGridCallback, this);

        globalGridMsg.info.resolution = resolution;
        globalGridMsg.info.width = map_width;
        globalGridMsg.info.height = map_height;
        //TODO: What do we choose as our original load time and origin?
        //globalGridMsg.info.map_load_time =
        //globalGridMsg.info.origin =

        //TODO: Why is int8 not recognized?
        auto* initial_map = new uint8[map_width * map_height];
        globalGridMsg.data = initial_map;
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