#include "inference.cuh"
#include "pch.hpp"
#include <opencv2/core/mat.hpp>
#include <ros/publisher.h>

namespace mrover {

    struct Detection {
        int class_id{0};
        std::string className{};
        float confidence{0.0};
        cv::Scalar color{};
        cv::Rect box{};
    };

    class ObjectDetectorNodelet : public nodelet::Nodelet {
        static const int NUM_CHANNELS = 3;
        static const int IMG_WIDTH = 1280;
        static const int IMG_HEIGHT = 720;

    private:
        ros::NodeHandle mNh, mPnh;

        //Inference inference;


        // Publishers

        ros::Publisher mDebugImgPub;
        ros::Publisher mDetectionData;


        // Subscribers

        ros::Subscriber mImgSub;

        // Preallocated cv::Mats
        cv::Mat imageBlob;

        dynamic_reconfigure::Server<mrover::ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::ObjectDetectorParamsConfig>::CallbackType mCallbackType;

        // Internal state

        cv::dnn::Net mNet;

        // Debug

        LoopProfiler mProfiler{"Object Detector"};

        void onInit() override;

        //DetectedObject convertToObjMsg(Detection& detection);


    public:
        ObjectDetectorNodelet() = default;

        ~ObjectDetectorNodelet() override = default;

        void imageCallback(sensor_msgs::ImageConstPtr const& msg);
    };
} // namespace mrover