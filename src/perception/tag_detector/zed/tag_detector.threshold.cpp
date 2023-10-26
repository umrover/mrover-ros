#include "tag_detector.hpp"

namespace mrover {

    void threshold(cv::InputArray in, cv::OutputArray out, int windowSize, double constant) {
        CV_Assert(windowSize >= 3);

        if (windowSize % 2 == 0) windowSize++; // win size must be odd
        cv::adaptiveThreshold(in, out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, windowSize, constant);
    }

    /**
     * Detect tags from raw image using OpenCV and calculate their screen space centers.
     * Tag pose information relative to the camera in 3D space is filled in when we receive point cloud data.
     *
     * @param msg
     */
    void TagDetectorNodelet::publishThresholdedImage() {
        cvtColor(mImg, mGrayImg, cv::COLOR_BGR2GRAY);

        // number of window sizes (scales) to apply adaptive thresholding
        int scaleCount = (mDetectorParams->adaptiveThreshWinSizeMax - mDetectorParams->adaptiveThreshWinSizeMin) / mDetectorParams->adaptiveThreshWinSizeStep + 1;

        // for each value in the interval of thresholding window sizes
        for (int scale = 0; scale < scaleCount; ++scale) {
            auto it = mThreshPubs.find(scale);
            if (it == mThreshPubs.end()) {
                ROS_INFO("Creating new publisher for thresholded scale %d", scale);
                std::tie(it, std::ignore) = mThreshPubs.emplace(scale, mNh.advertise<sensor_msgs::Image>("tag_detection_threshold_" + std::to_string(scale), 1));
            }
            auto& [_, publisher] = *it;

            if (publisher.getNumSubscribers() == 0) continue;

            int windowSize = mDetectorParams->adaptiveThreshWinSizeMin + scale * mDetectorParams->adaptiveThreshWinSizeStep;
            threshold(mGrayImg, mGrayImg, windowSize, mDetectorParams->adaptiveThreshConstant);

            mThreshMsg.header.seq = mSeqNum;
            mThreshMsg.header.stamp = ros::Time::now();
            mThreshMsg.header.frame_id = "zed2i_left_camera_frame";
            mThreshMsg.height = mGrayImg.rows;
            mThreshMsg.width = mGrayImg.cols;
            mThreshMsg.encoding = sensor_msgs::image_encodings::MONO8;
            mThreshMsg.step = mGrayImg.step;
            mThreshMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            size_t size = mThreshMsg.step * mThreshMsg.height;
            mThreshMsg.data.resize(size);
            std::uninitialized_copy(std::execution::par_unseq, mGrayImg.data, mGrayImg.data + size, mThreshMsg.data.begin());

            publisher.publish(mThreshMsg);
        }
    }

} // namespace mrover
