#include "object_detector.hpp"
#include "inference.h"
#include <cv_bridge/cv_bridge.h>
#include <mrover/DetectedObject.h>
#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/Image.h>
#include <string>


namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        //Inference

        inference = Inference("//home//jabra//Desktop//Rover//yolov8s.onnx", cv::Size(640, 640), "", true);
        //read ONNX file into this mNet, YOLOV8, second smallest one
        // Note(quintin): I downloaded this pt (PyTorch) model file from: https://github.com/ultralytics/assets/releases
        // TODO(percep/obj-detectr): make this configurable

        mImgSub = mNh.subscribe("/camera/left/image", 1, &ObjectDetectorNodelet::imageCallback, this);
        mDebugImgPub = mNh.advertise<sensor_msgs::Image>("/object_detector/debug_img", 1);
        mDetectionData = mNh.advertise<DetectedObject>("/object_detector/detected_object", 1);
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
