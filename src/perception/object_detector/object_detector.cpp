#include "object_detector.hpp"
#include "pch.hpp"
#include "inference_wrapper.hpp"

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        //Get the Handlers
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        

        //Create the inference wrapper
        mInferenceWrapper = InferenceWrapper(std::string("//home//john//Desktop//Rover//best.onnx"), cv::Size(640, 640), "");

        //Create the publishers and subscribers for the detected image and the debug image
        mImgSub = mNh.subscribe("/camera/left/points", 1, &ObjectDetectorNodelet::imageCallback, this);
        mDebugImgPub = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_img", 1);
        mDetectionData = mNh.advertise<DetectedObject>("/object_detector/detected_object", 1);

        //READ ROS PARAMETERS 

        //Create the Reference Frames
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed2i_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");

        //Read ROS Params
        mNh.param<int>("obj_increment_weight", mObjIncrementWeight, 2);
        mNh.param<int>("obj_decrement_weight", mObjDecrementWeight, 1);
        mNh.param<int>("obj_hitcount_threshold", mObjHitThreshold, 50);
        mNh.param<int>("obj_hitcount_max", mObjMaxHitcount, 60);

        //Initialize Object Hit Cout to Zeros
        mHitCount = 0;
    }
} // namespace mrover
