#include "object_detector.hpp"

namespace mrover {

    auto ObjectDetectorNodeletBase::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mNh.param<std::string>("camera_frame", mCameraFrame, "zed_left_camera_frame");
        mNh.param<std::string>("world_frame", mWorldFrame, "map");

        mPnh.param<int>("increment_weight", mObjIncrementWeight, 2);
        mPnh.param<int>("decrement_weight", mObjDecrementWeight, 1);
        mPnh.param<int>("hitcount_threshold", mObjHitThreshold, 5);
        mPnh.param<int>("hitcount_max", mObjMaxHitcount, 10);
        mPnh.param<std::string>("model_name", mModelName, "yolov8s_mallet_bottle");
        mPnh.param<float>("model_score_threshold", mModelScoreThreshold, 0.75);
        mPnh.param<float>("model_nms_threshold", mModelNmsThreshold, 0.5);

        mLearning = Learning{mModelName};

        mDebugImagePub = mNh.advertise<sensor_msgs::Image>("object_detection", 1);

        ROS_INFO_STREAM(std::format("Object detector initialized with model: {} and thresholds: {} and {}", mModelName, mModelScoreThreshold, mModelNmsThreshold));
    }

    auto StereoObjectDetectorNodelet::onInit() -> void {
        ObjectDetectorNodeletBase::onInit();

        mSensorSub = mNh.subscribe("/camera/left/points", 1, &StereoObjectDetectorNodelet::pointCloudCallback, this);
    }

    auto ImageObjectDetectorNodelet::onInit() -> void {
        ObjectDetectorNodeletBase::onInit();

        mPnh.param<float>("long_range_camera/fov", mCameraHorizontalFov, 80.0);

        mSensorSub = mNh.subscribe("long_range_camera/image", 1, &ImageObjectDetectorNodelet::imageCallback, this);

        mTargetsPub = mNh.advertise<ImageTargets>("objects", 1);
    }

} // namespace mrover
