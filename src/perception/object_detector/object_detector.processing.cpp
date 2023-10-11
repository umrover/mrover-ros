#include "object_detector.hpp"
#include <opencv2/dnn/dnn.hpp>

namespace mrover {

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        cv::Mat imageView{static_cast<int>(msg->width), static_cast<int>(msg->height), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};


        cv::Mat imageBlob = cv::dnn::blobFromImage(imageView);

        mNet.forward(imageBlob);


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
