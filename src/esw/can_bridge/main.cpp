#include "can_bridge.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "can_bridge");

    // Start the CAN Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CanNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif
