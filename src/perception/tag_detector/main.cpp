#include "tag_detector.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::TagDetectorNodelet, nodelet::Nodelet)

#else

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_detector");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/TagDetectorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif
