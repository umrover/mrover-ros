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
        mNh.param<int>("obj_increment_weight", mObjIncrementWeight, 1);
        mNh.param<int>("obj_decrement_weight", mObjDecrementWeight, 5);
        mNh.param<int>("obj_decrement_threshold", mObjHitThreshold, 100);
        mNh.param<int>("obj_max_hitcount", mObjMaxHitcount, 200);

        //Initialize Object Hit Cout to Zeros
        mHitCount = 0;
    }
} // namespace mrover


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/ObjectDetectorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ObjectDetectorNodelet, nodelet::Nodelet)
#endif
