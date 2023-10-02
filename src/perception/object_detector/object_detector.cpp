#include "object_detector.hpp"

namespace mrover {

    void ObjectDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        // TODO(percep/obj-detectr): paramaterize this
        mNet = cv::dnn::readNetFromTorch("/home/quintin/Downloads/yolov8n.pt");

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
