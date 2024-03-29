#include "object_detector.hpp"
#include "inference_wrapper.hpp"

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        //Loop profiler param
        mNh.param<bool>("enable_loop_profiler", mEnableLoopProfiler, false);

        //TF Params
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");
        mNh.param<std::string>("world_frame", mMapFrame, "map");

        //Hit count params
        mNh.param<int>("obj_increment_weight", mObjIncrementWeight, 2);
        mNh.param<int>("obj_decrement_weight", mObjDecrementWeight, 1);
        mNh.param<int>("obj_hitcount_threshold", mObjHitThreshold, 5);
        mNh.param<int>("obj_hitcount_max", mObjMaxHitcount, 10);

        //Model Params
        mNh.param<std::string>("model_name", mModelName, "yolov8n_mallet_bottle_better");

        //Init Model Path
        std::filesystem::path packagePath = ros::package::getPath("mrover");
        std::filesystem::path modelPath = packagePath / "data" / (mModelName + ".onnx");

        //Wrapper
        mInferenceWrapper = InferenceWrapper{modelPath, mModelName};

        //Pub sub
        mImgSub = mNh.subscribe("/camera/left/points", 1, &ObjectDetectorNodelet::imageCallback, this);
        mDebugImgPub = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_img", 1);
    }

} // namespace mrover