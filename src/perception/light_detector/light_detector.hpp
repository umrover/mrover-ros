#pragma once
#include "pch.hpp"

namespace mrover {
	class LightDetector : public nodelet::Nodelet {
	private:
		ros::NodeHandle mNh, mPnh;

		cv::Mat mImg;

		// Thresholding Variables
		cv::Mat mThresholdedImg;
		cv::Scalar mUpperBound{255, 60, 60};
		cv::Scalar mLowerBound{125, 0, 0};

		// Pub Sub
		ros::Subscriber imgSub;
		ros::Publisher imgPub;

		auto imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

		auto static convertPointCloudToRGB(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat const& image) -> void;

		auto publishDetectedObjects(cv::InputArray image) -> void;

	public:
		auto onInit() -> void override;

		LightDetector() = default;

		~LightDetector() override = default;
	}; // LightDetector
} // mrover

