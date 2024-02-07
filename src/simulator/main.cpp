#include "simulator.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::SimulatorNodelet, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "simulator");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/SimulatorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif
