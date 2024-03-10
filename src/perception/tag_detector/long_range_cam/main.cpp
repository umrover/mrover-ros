#include "long_range_tag_detector.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeTagDetectorNodelet, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "long_range_tag_detector");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/LongRangeTagDetectorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif