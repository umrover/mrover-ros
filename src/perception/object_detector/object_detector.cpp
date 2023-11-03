#include "object_detector.hpp"
//#include "inference.h"
#include <cv_bridge/cv_bridge.h>

#include <mrover/DetectedObject.h>
//#include <mrover/DetectedObjects.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/Image.h>
#include <string>


namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        inference = Inference("//home//jabra//Desktop//Rover//yolov8s.onnx", cv::Size(640, 640), "");
        //read ONNX file into this mNet, YOLOV8, second smallest one
        // Note(quintin): I downloaded this pt (PyTorch) model file from: https://github.com/ultralytics/assets/releases
        // TODO(percep/obj-detectr): make this configurable

        mImgSub = mNh.subscribe("/camera/left/image", 1, &ObjectDetectorNodelet::imageCallback, this);
        mDebugImgPub = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_img", 1);
        mDetectionData = mNh.advertise<DetectedObjects>("/object_detector/detected_object", 1);

        this->imageBlob = cv::Mat{1, ObjectDetectorNodelet::NUM_CHANNELS, ObjectDetectorNodelet::IMG_WIDTH, ObjectDetectorNodelet::IMG_HEIGHT, CV_32F};
    }

    void ObjectDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        //Ensure a valid message was received
        assert(msg);

        //Get a CV::Mat image view
        cv::Mat imageView{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        cv::dnn::blobFromImage(imageView, this->imageBlob);

        //Run Image Detections
        //Return type of Detection struct
        std::vector<Detection> detections = this->inference.doDetections(imageView);

        //just an array of DetectedObject
        DetectedObjects detectionsMessage;

        detectionsMessage.num_detections = static_cast<int>(detections.size());

        //Convert Vector of Detections to Output Message
        for (Detection& detection: detections) {
            DetectedObject objMsg = convertToObjMsg(detection);
            detectionsMessage.push_back(objMsg);
        }

        // detectionsMessage.header.seq = mSeqNum;
        detectionsMessage.header.stamp = ros::Time::now();
        // detectionsMessage.header.frame_id = "frame"

        this->publisher
    }

    int main(int argc, char** argv) {
        ros::init(argc, argv, "object_detector");

        // Start the ZED Nodelet
        nodelet::Loader nodelet;
        nodelet.load(ros::this_node::getName(), "mrover/ObjectDetectorNodelet", ros::names::getRemappings(), {});

        ros::spin();

        return EXIT_SUCCESS;
    }

} // namespace mrover

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ObjectDetectorNodelet, nodelet::Nodelet)
#endif
