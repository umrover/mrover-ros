#include "click_ik.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ClickIkNodelet, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "click_ik");

    // Start the ClickIK Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/ClickIkNodelet", ros::names::getRemappings(), {});
    ros::spin();

    return EXIT_SUCCESS;
}

#endif
