#pragma once
#include "pch.hpp"

namespace mrover {
	class LightDetector : public nodelet::Nodelet {
	private:
		ros::NodeHandle mNh, mPnh;

		cv::Mat mImgRGB;
		cv::Mat mImgHSV;

		cv::Mat mOutputImage;


		// Thresholding Variables
		cv::Mat mThresholdedImg;
		cv::Scalar mUpperBound{255, 70, 70};
		cv::Scalar mLowerBound{80, 10, 10};

		cv::Mat mErodedImg;

		cv::Mat mDialtedImg;

		// Pub Sub
		ros::Subscriber imgSub;
		ros::Publisher imgPub;

		auto imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

		auto static convertPointCloudToRGB(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat const& image) -> void;

		auto publishDetectedObjects(cv::InputArray image) -> void;

		auto static rgb_to_hsv(cv::Vec3b const& rgb) -> cv::Vec3d;

	public:
		auto onInit() -> void override;

		LightDetector() = default;

		~LightDetector() override = default;
	}; // LightDetector
} // mrover

