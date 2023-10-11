#include "object_detector.hpp"

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        //read ONNX file into this mNet, YOLOV8, second smallest one
        // Note(quintin): I downloaded this pt (PyTorch) model file from: https://github.com/ultralytics/assets/releases
        // TODO(percep/obj-detectr): make this configurable
        mNet = cv::dnn::readNetFromTorch("/home/marshallvielmetti/Downloads/yolov8s.pt");

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
