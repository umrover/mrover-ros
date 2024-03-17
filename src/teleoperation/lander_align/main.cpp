#include "lander_align.hpp"

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "lander_align");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/LanderAlignNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LanderAlignNodelet, nodelet::Nodelet)
#endif
