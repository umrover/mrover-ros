#include "nv_gst_h265_enc.hpp"

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::NvGstH265EncNodelet, nodelet::Nodelet)

#else

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "nv_gst_h265_enc");

    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/NvGstH265EncNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif