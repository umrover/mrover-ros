#pragma once
#include "pch.hpp"
#include <unordered_map>

class PairHash{
	public:
		auto operator()(std::pair<int, int> const& p) const -> int{
			return p.first * 10000000 + p.second;
		}
};

namespace mrover {
	class LightDetector : public nodelet::Nodelet {
	private:
		ros::NodeHandle mNh, mPnh;

		cv::Mat mImgRGB;
		cv::Mat mImgHSV;

		cv::Mat mOutputImage;

		int SPIRAL_SEARCH_DIM{};

		double mImmediateLightRange{};

		std::unordered_map<std::pair<int, int>, int, PairHash> mHitCounts;

		int mHitIncrease{};
		int mHitDecrease{};
		int mHitMax{};
		int mPublishThreshold{};

		// TF variables
		tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
		std::string mCameraFrame;
        std::string mWorldFrame;

		// Thresholding Variables
		cv::Mat mThresholdedImg;
		cv::Vec3d mUpperBound;
		cv::Vec3d mLowerBound;

		cv::Mat mErodedImg;

		cv::Mat mDialtedImg;

		// Pub Sub
		ros::Subscriber imgSub;
		ros::Publisher imgPub;

		auto imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

		auto static convertPointCloudToRGB(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat const& image) -> void;

		auto publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void;

		auto static rgb_to_hsv(cv::Vec3b const& rgb) -> cv::Vec3d;

		auto spiralSearchForValidPoint(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d>;

		void increaseHitCount(std::optional<SE3d> const& light);

		void decreaseHitCounts();

		auto getHitCount(std::optional<SE3d> const& light) -> int;


	public:
		auto onInit() -> void override;

		LightDetector() = default;

		~LightDetector() override = default;
	}; // LightDetector
} // mrover

