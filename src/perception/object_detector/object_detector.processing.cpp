#include "object_detector.hpp"
#include "inference.h"
#include <mrover/DetectedObject.h>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>


namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        //assert(msg);
        //assert(msg->height > 0);
        //assert(msg->width > 0);
        cv::VideoCapture cap(0);
        cv::Mat rawImg; // = cv::imread("//home//jabra//Downloads//Water.jpg");
        cap.read(rawImg);
        cv::Mat sizedImg;
        cv::resize(rawImg, sizedImg, cv::Size(640, 640));
        //cv::Mat imageView{static_cast<int>(msg->width), static_cast<int>(msg->height), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        std::vector<Detection> detections = inference.runInference(sizedImg);
        if (!detections.empty()) {
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
            msgData.heading = 0;


            //Put the rectangle on the image
            cv::rectangle(sizedImg, box, cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);

            //Put the text on the image
            cv::Point text_position(80, 80);
            int font_size = 1;
            cv::Scalar font_Color(0, 0, 0);
            int font_weight = 2;
            putText(sizedImg, msgData.object_type, text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_Color, font_weight); //Putting the text in the matrix//

            //Show the image
            cv::imshow("obj detector", sizedImg);
            cv::waitKey(1);
            //Print the type of objected detected
            ROS_INFO(firstDetection.className.c_str());
        }
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
