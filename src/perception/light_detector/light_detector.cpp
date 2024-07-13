#include "light_detector.hpp"
#include <opencv2/core/matx.hpp>

namespace mrover{

	auto LightDetector::onInit() -> void{
		ROS_INFO_STREAM("Light Detector Initializing");
		mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

		int upperBoundH = 0;
		int upperBoundS = 0;
		int upperBoundV = 0;
		int lowerBoundH = 0;
		int lowerBoundS = 0;
		int lowerBoundV = 0;

        mPnh.param<int>("/light_detector/upper_bound_h", upperBoundH, 0);
        mPnh.param<int>("/light_detector/upper_bound_s", upperBoundS, 0);
        mPnh.param<int>("/light_detector/upper_bound_v", upperBoundV, 0);
        mPnh.param<int>("/light_detector/lower_bound_h", lowerBoundH, 0);
        mPnh.param<int>("/light_detector/lower_bound_s", lowerBoundS, 0);
        mPnh.param<int>("/light_detector/lower_bound_v", lowerBoundV, 0);

		mUpperBound = cv::Vec3d(upperBoundH, upperBoundS, upperBoundV);
		mLowerBound = cv::Vec3d(lowerBoundH, lowerBoundS, lowerBoundV);

		imgSub = mNh.subscribe("/camera/left/points", 1, &LightDetector::imageCallback, this);
		imgPub = mNh.advertise<sensor_msgs::Image>("/light_detector/img", 1);
	}

} // mrover
