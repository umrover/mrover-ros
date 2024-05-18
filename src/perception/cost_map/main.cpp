#include "cost_map.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CostMapNodelet, nodelet::Nodelet)

#else

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap");

    // Start Costmap Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CostMapNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif
