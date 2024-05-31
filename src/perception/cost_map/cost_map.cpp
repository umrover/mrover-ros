#include "cost_map.hpp"

namespace mrover {

    auto CostMapNodelet::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mResolution = mPnh.param<float>("resolution", 0.5);
        mSize = mPnh.param<float>("size", 32);
        mMapFrame = mPnh.param<std::string>("map_frame", "map");
        mNearClip = mPnh.param<float>("near_clip", 0.2);
        mFarClip = mPnh.param<float>("far_clip", 3);
        mZPercent = mPnh.param<float>("z_percent", 0.1);
        mAlpha = mPnh.param<float>("alpha", 0.05);
        mZThreshold = mPnh.param<float>("z_threshold", 0);

        mServer = mNh.advertiseService("move_cost_map", &CostMapNodelet::moveCostMapCallback, this);

        mCostMapPub = mNh.advertise<nav_msgs::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mPcSub = mNh.subscribe("camera/left/points", 1, &CostMapNodelet::pointCloudCallback, this);
        mImuSub = mNh.subscribe<sensor_msgs::Imu>("imu/data", 1, [this](sensor_msgs::ImuConstPtr const&) {
            mLastImuTime = ros::Time::now();
        });

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