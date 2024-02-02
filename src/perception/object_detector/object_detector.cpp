#include "object_detector.hpp"
#include "inference_wrapper.hpp"
#include "pch.hpp"

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        //Get the Handlers
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        std::filesystem::path packagePath = ros::package::getPath("mrover");
        std::filesystem::path modelPath = packagePath / "data" / "yolov8n_mallet_bottle_better.onnx";

        mInferenceWrapper = InferenceWrapper{modelPath};

        //Create the publishers and subscribers for the detected image and the debug image
        mImgSub = mNh.subscribe("/camera/left/points", 1, &ObjectDetectorNodelet::imageCallback, this);
        mDebugImgPub = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_img", 1);

        //Create the Reference Frames
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed2i_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");

        //Read ROS Params
        mNh.param<int>("obj_increment_weight", mObjIncrementWeight, 2);
        mNh.param<int>("obj_decrement_weight", mObjDecrementWeight, 1);
        mNh.param<int>("obj_hitcount_threshold", mObjHitThreshold, 5);
        mNh.param<int>("obj_hitcount_max", mObjMaxHitcount, 10);

        //Initialize Object Hit Cout to Zeros
        mObjectHitCounts = {0, 0};
    }
} // namespace mrover
