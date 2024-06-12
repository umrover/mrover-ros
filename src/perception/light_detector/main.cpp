#include "light_detector.hpp"
#include "pch.hpp"

// Load LightDetector nodelet if we are dynamically loading it
#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LightDetector, nodelet::Nodelet)

#endif

auto main(int argc, char* argv[]) -> int{
	ros::init(argc, argv, "light_detector");

	nodelet::Loader loader;
    loader.load(ros::this_node::getName(), "mrover/LightDetector", ros::names::getRemappings(), {});

    ros::spin();

	return EXIT_SUCCESS;
}
