#include "object_detector.hpp"
#include "inference.h"
#include <cv_bridge/cv_bridge.h>
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
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>


namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        cv::Mat sizedImg{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        cv::cvtColor(sizedImg, sizedImg, cv::COLOR_BGRA2BGR);

        cv::imshow("heheh", sizedImg);
        cv::waitKey(1);

        std::vector<Detection> detections = inference.runInference(sizedImg);
        if (!detections.empty()) {
            Detection firstDetection = detections[0];

            float classConfidence = 0.0;
            cv::Rect box = firstDetection.box;

            DetectedObject msgData;
            msgData.object_type = firstDetection.className;
            msgData.detection_confidence = classConfidence;

            msgData.xBoxPixel = static_cast<float>(box.x);
            msgData.yBoxPixel = static_cast<float>(box.y);
            msgData.width = static_cast<float>(box.width);
            msgData.height = static_cast<float>(box.height);

            //Get the heading
            /*
            float objectHeading;
            float zedFOV = 54; //54 @ 720; 42 @ 1080
            float fovPerPixel = (float) zedFOV / (float) (msg->width);
            float xCenter = (float) box.x + ((float) box.width) / 2 - ((float) msg->width) / 2;
            objectHeading = xCenter * fovPerPixel;
            msgData.heading = objectHeading;
            */

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
            ROS_INFO_STREAM(firstDetection.className.c_str());

            // if (mDebugImgPub.getNumSubscribers() > 0) {
            //     // Create sensor msg image
            //     sensor_msgs::ImageConstPtr newMessage;

            //     newMessage->height = sizedImg.getHeight();
            //     newMessage->width = sizedImg.getWidth();
            //     newMessage->encoding = sensor_msgs::image_encodings::BGRA8;
            //     newMessage->step = sizedImg.getStepBytes();
            //     newMessage->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            //     auto* bgrGpuPtr = bgra.getPtr<sl::uchar1>(sl::MEM::GPU);
            //     size_t size = msg->step * msg->height;
            //     msg->data.resize(size);

            //     checkCudaError(cudaMemcpy(msg->data.data(), bgrGpuPtr, size, cudaMemcpyDeviceToHost));
            // }
            // Publish to
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
