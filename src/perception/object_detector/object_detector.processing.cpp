#include "object_detector.hpp"
#include "inference.h"
#include <mrover/DetectedObject.h>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>


namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        cv::Mat imageView = cv::imread("/home/jabra/Downloads/Water.jpg");
        //cv::Mat imageView{static_cast<int>(msg->width), static_cast<int>(msg->height), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        std::vector<Detection> detections = inference.runInference(imageView);

        // struct Detection {
        //     int class_id{0};
        //     std::string className{};
        //     float confidence{0.0};
        //     cv::Scalar color{};
        //     cv::Rect box{};
        // };

        Detection firstDetection = detections[0];

        float classConfidence = 0.0;
        cv::Rect box = firstDetection.box;


        DetectedObject msgData;
        msgData.object_type = firstDetection.className;
        msgData.detection_confidence = classConfidence;
        msgData.xBoxPixel = (float) box.x;
        msgData.yBoxPixel = (float) box.y;
        msgData.width = (float) box.width;
        msgData.height = (float) box.height;

        //Get the heading
        float objectHeading;
        float zedFOV = 54; //54 @ 720; 42 @ 1080
        float fovPerPixel = (float) zedFOV / (float) (msg->width);
        float xCenter = (float) box.x + ((float) box.width) / 2 - ((float) msg->width) / 2;
        objectHeading = xCenter * fovPerPixel;
        msgData.heading = objectHeading;


        //Look at yolov8 documentation for the output matrix

        /*
         * TODO(percep/obj-detector):
         * 0. Google "OpenCV DNN example." View a couple of tutorials. Most will be in Python but the C++ API is similar.
         * 1. Use cv::dnn::blobFromImage to convert the image to a blob
         * 2. Use mNet.forward to run the network
         * 3. Parse the output of the network to get the bounding boxes
         * 4. Publish the bounding boxes
         */
    }

} // namespace mrover
