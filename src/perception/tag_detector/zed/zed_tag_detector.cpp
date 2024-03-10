#include "zed_tag_detector.hpp"

namespace mrover {

    auto StereoTagDetectorNodelet::specificOnInit() -> void {
        mNh.param<bool>("use_odom_frame", mUseOdom, false);
        mNh.param<std::string>("odom_frame", mOdomFrameId, "odom");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");

        mPnh.param<int>("dictionary", dictionaryNumber, static_cast<int>(cv::aruco::DICT_4X4_50));
        mPnh.param<int>("min_hit_count_before_publish", mMinHitCountBeforePublish, 5);
        mPnh.param<int>("max_hit_count", mMaxHitCount, 5);
        mPnh.param<int>("tag_increment_weight", mTagIncrementWeight, 2);
        mPnh.param<int>("tag_decrement_weight", mTagDecrementWeight, 1);

        mImgPub = mNh.advertise<sensor_msgs::Image>("tag_detection", 1);

        mPcSub = mNh.subscribe("camera/left/points", 1, &StereoTagDetectorNodelet::pointCloudCallback, this);

    }    // namespace mrover

}
