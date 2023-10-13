#include "object_detector.hpp"
#include "inference.h"
#include <opencv2/core/types.hpp>

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        //Inference

        inference = Inference("/home/jabra/Downloads/yolov8s.pt", cv::Size(720, 480), "", false);

        //read ONNX file into this mNet, YOLOV8, second smallest one
        // Note(quintin): I downloaded this pt (PyTorch) model file from: https://github.com/ultralytics/assets/releases
        // TODO(percep/obj-detectr): make this configurable

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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ObjectDetectorNodelet, nodelet::Nodelet)
