#include "long_range_cam.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeCamNodelet, nodelet::Nodelet)

#else

int main(int argc, char** argv) {
    ros::init(argc, argv, "usb_camera");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    std::cout << ros::this_node::getName() << std::endl;
    nodelet.load(ros::this_node::getName(), "mrover/LongRangeCamNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif