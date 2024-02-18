#include "click_ik.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ClickIkNodelet, nodelet::Nodelet)

// #else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "click_ik");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/ClickIkNodelet", ros::names::getRemappings(), {});
    // Start ActionServer
    ros::NodeHandle n;
    // Server server(n, "do_click_ik", boost::bind(&execute, _1, &server), false);
    // server.start();

    ros::spin();

    return EXIT_SUCCESS;
}

#endif
