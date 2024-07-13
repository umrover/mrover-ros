#include "light_detector.hpp"

namespace mrover{

	auto LightDetector::onInit() -> void{
		ROS_INFO_STREAM("Light Detector Initializing");
		mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

		int upperBoundR = 0;
		int upperBoundG = 0;
		int upperBoundB = 0;
		int lowerBoundR = 0;
		int lowerBoundG = 0;
		int lowerBoundB = 0;

        mPnh.param<int>("light_detector/upper_bound_r", upperBoundR, 0);
        mPnh.param<int>("light_detector/upper_bound_g", upperBoundG, 0);
        mPnh.param<int>("light_detector/upper_bound_b", upperBoundB, 0);
        mPnh.param<int>("light_detector/lower_bound_r", lowerBoundR, 0);
        mPnh.param<int>("light_detector/lower_bound_g", lowerBoundG, 0);
        mPnh.param<int>("light_detector/lower_bound_b", lowerBoundB, 0);

		mUpperBound = cv::Scalar(upperBoundR, upperBoundG, upperBoundB);
		mUpperBound = cv::Scalar(lowerBoundR, lowerBoundG, lowerBoundB);

		imgSub = mNh.subscribe("/camera/left/points", 1, &LightDetector::imageCallback, this);
		imgPub = mNh.advertise<sensor_msgs::Image>("/light_detector/img", 1);
	}

} // mrover
