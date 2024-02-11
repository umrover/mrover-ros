#include "long_range_tag_detector.hpp"
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <sensor_msgs/Image.h>

namespace mrover {

    /**
     * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
     *
     * @param msg   image message
     */
    void LongRangeTagDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
        if (!mEnableDetections) return;

        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        //Store image contents to member variables
        cv::Mat cvImage = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};
        cvImage.copyTo(mImg);

        // Detect tags
        cv::aruco::detectMarkers(mImg, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);

        // Only publish the tags if the topic has subscribers
        if (mPublishImages && mImgPub.getNumSubscribers()) {
            // Draw the tags on the image using OpenCV
            publishTagsOnImage();
        }

        // Publish tags to navigation
        LongRangeTags tagsMessage;

        for (size_t i = 0; i < mImmediateIds.size(); ++i) {
            LongRangeTag newTagMessage;

            //Fill in fields
            newTagMessage.id = mImmediateIds[i];
            newTagMessage.bearing = getTagBearing(mImmediateCorners[i]);
            //Add to back of tagsMessage
            tagsMessage.longRangeTags.push_back(newTagMessage);
        }

        //tagsMessage should be a vector of LongRangeTag messages
        mLongRangeTagsPub.publish(tagsMessage);

        size_t detectedCount = mImmediateIds.size();
        NODELET_INFO_COND(!mPrevDetectedCount.has_value() ||
                                  detectedCount != mPrevDetectedCount.value(),
                          "Detected %zu markers", detectedCount);
        mPrevDetectedCount = detectedCount;

        ++mSeqNum;
    }

    float LongRangeTagDetectorNodelet::getTagBearing(std::vector<cv::Point2f>& tagCorners) const {
        // Takes the average of the corners
        cv::Point2f center = std::reduce(tagCorners.begin(), tagCorners.end()) / static_cast<float>(tagCorners.size());
        float bearing = -1 * (center.x - (mImg.cols / 2.0)) / mImg.cols * mLongRangeFov;
        return bearing;
    }

    void LongRangeTagDetectorNodelet::publishTagsOnImage() {
        cv::aruco::drawDetectedMarkers(mImg, mImmediateCorners, mImmediateIds);
        mImgMsg.header.seq = mSeqNum;
        mImgMsg.header.stamp = ros::Time::now();
        mImgMsg.header.frame_id = "long_range_cam_frame";
        mImgMsg.height = mImg.rows;
        mImgMsg.width = mImg.cols;
        mImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
        mImgMsg.step = mImg.step;
        mImgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        size_t size = mImgMsg.step * mImgMsg.height;
        mImgMsg.data.resize(size);
        std::uninitialized_copy(std::execution::par_unseq, mImg.data, mImg.data + size, mImgMsg.data.begin());
        mImgPub.publish(mImgMsg);
    }

} // namespace mrover