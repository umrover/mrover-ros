#include "ir_camera.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::IRCamera, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "ir_camera");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/IRCamera", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif