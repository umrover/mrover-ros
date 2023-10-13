#include "inference.h"
#include "pch.hpp"

namespace mrover {

    class ObjectDetectorNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        Inference inference;

        // Publishers

        ros::Subscriber mImgSub;

        // Subscribers

        dynamic_reconfigure::Server<mrover::ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::ObjectDetectorParamsConfig>::CallbackType mCallbackType;

        // Internal state

        cv::dnn::Net mNet;

        // Debug

        LoopProfiler mProfiler{"Object Detector"};

        void onInit() override;

    public:
        ObjectDetectorNodelet() = default;

        ~ObjectDetectorNodelet() override = default;

        void imageCallback(sensor_msgs::ImageConstPtr const& msg);
    };

} // namespace mrover
