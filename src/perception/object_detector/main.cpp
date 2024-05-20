#include "object_detector.hpp"

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "object_detector");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/StereoObjectDetectorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::StereoObjectDetectorNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(mrover::ImageObjectDetectorNodelet, nodelet::Nodelet)
#endif