#include "pch.hpp"

#include "inference_wrapper.hpp"
#include <opencv2/core/mat.hpp>
#include <ros/publisher.h>

namespace mrover {

    struct Detection {
        int class_id{0};
        std::string className;
        float confidence{0.0};
        cv::Scalar color{};
        cv::Rect box{};
    };

    class ObjectDetectorNodelet : public nodelet::Nodelet {

        constexpr static int NUM_CHANNELS = 3;
        constexpr static int IMG_WIDTH = 1280;
        constexpr static int IMG_HEIGHT = 720;

        std::vector<std::string> classes{"Hammer"};//{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};


        ros::NodeHandle mNh, mPnh;

        //Inference inference;
        InferenceWrapper mInferenceWrapper;

        // Publishers

        ros::Publisher mDebugImgPub;
        ros::Publisher mDetectionData;

        // Subscribers

        ros::Subscriber mImgSub;

        // Preallocated cv::Mats
        cv::Mat mImageBlob;

        dynamic_reconfigure::Server<ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<ObjectDetectorParamsConfig>::CallbackType mCallbackType;

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