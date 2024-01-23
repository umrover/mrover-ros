#include "image_capture.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_capture");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/ImageCaptureNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ImageCaptureNodelet, nodelet::Nodelet)
#endif
