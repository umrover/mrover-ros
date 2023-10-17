#include "object_detector.hpp"
#include "inference.h"
#include <mrover/DetectedObject.h>
#include <opencv2/core/types.hpp>
#include <sensor_msgs/Image.h>
#include <string>

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        //Inference

        inference = Inference("//home//jabra//Desktop//Rover//yolov8s.onnx", cv::Size(640, 640), "", false);
        //read ONNX file into this mNet, YOLOV8, second smallest one
        // Note(quintin): I downloaded this pt (PyTorch) model file from: https://github.com/ultralytics/assets/releases
        // TODO(percep/obj-detectr): make this configurable

        cv::Mat rawImg = cv::imread("//home//jabra//Downloads//Water.jpg");
        cv::Mat sizedImg;
        cv::resize(rawImg, sizedImg, cv::Size(640, 640));
        //cv::Mat imageView{static_cast<int>(msg->width), static_cast<int>(msg->height), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        std::vector<Detection> detections = inference.runInference(sizedImg);

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
        ROS_INFO(firstDetection.className.c_str());

        mImgSub = mNh.subscribe("image", 1, &ObjectDetectorNodelet::imageCallback, this);
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/ObjectDetectorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ObjectDetectorNodelet, nodelet::Nodelet)
#endif
