#include "long_range_tag_detector.hpp"

namespace mrover {

    /**
     * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
     *
     * @param msg   Image message
     */
    auto LongRangeTagDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
        if (!mEnableDetections) return;

        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        assert(msg->encoding == sensor_msgs::image_encodings::BGRA8);

        //Store image contents to member variables
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())};
        cv::cvtColor(bgraImage, mImg, cv::COLOR_BGRA2BGR);

        // Detect tags
        cv::aruco::detectMarkers(mImg, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);

        // Only publish the tags if the topic has subscribers
        if (mPublishImages && mImgPub.getNumSubscribers()) {
            // Draw the tags on the image using OpenCV
            publishTagsOnImage();
        }

        // Publish tags to navigation
        LongRangeTags tagsMessage;

        for (std::size_t i = 0; i < mImmediateIds.size(); ++i) {
            LongRangeTag newTagMessage;

            newTagMessage.id = mImmediateIds[i];
            newTagMessage.bearing = getTagBearing(mImmediateCorners[i]);
            tagsMessage.longRangeTags.push_back(newTagMessage);
        }

        mLongRangeTagsPub.publish(tagsMessage);

        std::size_t detectedCount = mImmediateIds.size();
        NODELET_INFO_COND(!mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value(), "Detected %zu markers", detectedCount);
        mPrevDetectedCount = detectedCount;
    }

    auto LongRangeTagDetectorNodelet::getTagBearing(std::vector<cv::Point2f>& tagCorners) const -> float {
        // Takes the average of the corners
        cv::Point2f center = std::reduce(tagCorners.begin(), tagCorners.end()) / static_cast<float>(tagCorners.size());
        float bearing = -1.0f * (center.x - static_cast<float>(mImg.cols) / 2.0f) / static_cast<float>(mImg.cols) * mLongRangeFov;
        return bearing;
    }

    auto LongRangeTagDetectorNodelet::publishTagsOnImage() -> void {
        cv::aruco::drawDetectedMarkers(mImg, mImmediateCorners, mImmediateIds);
        mImgMsg.header.stamp = ros::Time::now();
        mImgMsg.header.frame_id = "long_range_cam_frame";
        mImgMsg.height = mImg.rows;
        mImgMsg.width = mImg.cols;
        mImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
        mImgMsg.step = mImg.step;
        mImgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        std::size_t size = mImgMsg.step * mImgMsg.height;
        mImgMsg.data.resize(size);
        std::memcpy(mImgMsg.data.data(), mImg.data, size);
        mImgPub.publish(mImgMsg);
    }

} // namespace mrover