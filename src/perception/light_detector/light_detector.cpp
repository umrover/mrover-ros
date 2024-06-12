#include "light_detector.hpp"

namespace mrover{

	auto LightDetector::onInit() -> void{
		ROS_INFO_STREAM("Light Detector Initializing");
		mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

		imgSub = mNh.subscribe("/camera/left/points", 1, &LightDetector::imageCallback, this);
		imgPub = mNh.advertise<sensor_msgs::Image>("/light_detector/img", 1);
	}

} // mrover
