#include "gst_websocket_streamer.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::GstWebsocketStreamerNodelet, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "gst_websocket_streamer");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/GstWebsocketStreamerNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif