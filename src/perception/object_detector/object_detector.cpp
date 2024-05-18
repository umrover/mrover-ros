#include "object_detector.hpp"
#include "learning.hpp"

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        //TF Params
        mNh.param<std::string>("camera_frame", mCameraFrame, "zed_left_camera_frame");
        mNh.param<std::string>("world_frame", mWorldFrame, "map");

        // Hit count params
        mNh.param<int>("obj_increment_weight", mObjIncrementWeight, 2);
        mNh.param<int>("obj_decrement_weight", mObjDecrementWeight, 1);
        mNh.param<int>("obj_hitcount_threshold", mObjHitThreshold, 5);
        mNh.param<int>("obj_hitcount_max", mObjMaxHitcount, 10);

        // Point Cloud
        // Model Params
        mNh.param<std::string>("model_name_point_cloud", mModelNamePC, "yolov8n_mallet_bottle_better");

        // Learning
        mLearningPC = Learning{mModelNamePC};

        // Pub and Sub
        mImgSubPC = mNh.subscribe("/camera/left/points", 1, &ObjectDetectorNodelet::pointCloudCallback, this);
        mDebugImgPubPC = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_point_cloud_img", 1);

        // Image TODO(LONG RANGE CAMERA): update the pub sub section to have the correct
        // Model Params
        mNh.param<std::string>("model_name_image", mModelNameIMG, "yolov8n_mallet_bottle_better");

        // Learning
        mLearningIMG = Learning{mModelNameIMG};

        // Pub and Sub
        mImgSubIMG = mNh.subscribe("/long_range_image", 1, &ObjectDetectorNodelet::imageCallback, this);
        mDebugImgPubIMG = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_image_img", 1);
    }

} // namespace mrover