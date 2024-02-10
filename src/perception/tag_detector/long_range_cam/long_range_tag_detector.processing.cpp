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

        //Store image contents to member variables
        updateImageMatrices(msg);

        // 1. Detect tags
        runTagDetection();

        // 2. Update the hit counts of the tags in the mTags map
        updateHitCounts();

        // 3. We only want to publish the tags if the topic has subscribers
        if (mPublishImages && mImgPub.getNumSubscribers()) {
            // Draw the tags on the image using OpenCV
            publishTagsOnImage();
        }

        //Publish all tags that meet threshold
        publishPermanentTags();

        size_t detectedCount = mImmediateIds.size();
        NODELET_INFO_COND(!mPrevDetectedCount.has_value() ||
                                  detectedCount != mPrevDetectedCount.value(),
                          "Detected %zu markers", detectedCount);
        mPrevDetectedCount = detectedCount;

        //PUBLISH TAGS
        ++mSeqNum;
    }

    //HELPER FUNCTIONS

    void LongRangeTagDetectorNodelet::updateImageMatrices(sensor_msgs::ImageConstPtr const& msg) {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        // TODO: Modify when going back to real cam
        cv::Mat cvImage = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, const_cast<uint8_t*>(msg->data.data())};

        //Store to mImg member variable - unsure if necessary but need it to persist
        cvImage.copyTo(mImg);

        publishTagsOnImage();
        // TODO: Set the grayImage if neccesary
    }

    void LongRangeTagDetectorNodelet::runTagDetection() {
        cv::aruco::detectMarkers(mImg, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);
    }


    void LongRangeTagDetectorNodelet::updateHitCounts() {
        //loop through all identified IDs
        for (size_t i = 0; i < mImmediateIds.size(); i++) {
            updateNewlyIdentifiedTags(i);
        }

        //Now decrement all the hitcounts for tags that were not updated
        // Set updated status to false
        for (auto it = mTags.begin(); it != mTags.end();) {
            LongRangeTagStruct& currentTag = it->second;
            if (currentTag.updated) {
                currentTag.updated = false;
                it++;
            } else {
                //Decrement weight of undetected tags
                currentTag.hitCount -= mTagDecrementWeight;

                //if the value has fallen belown the minimum, remove it
                if (currentTag.hitCount <= mTagRemoveWeight) {
                    it = mTags.erase(it);
                } else {
                    it++;
                }
            }
        }


        //decrement non updated & set updated status to false
    }

    LongRangeTagStruct LongRangeTagDetectorNodelet::createLrt(int tagId, std::vector<cv::Point2f>& tagCorners) {
        //TODO: Modify the LRT struct and this function to assign a bearing instead of an imageCenter
        LongRangeTagStruct lrt;

        lrt.hitCount = mBaseHitCount; //Start at base hit count value
        lrt.id = tagId;
        lrt.updated = true;

        lrt.imageCenter = getNormedTagCenterOffset(tagCorners);
        return lrt;
    }

    void LongRangeTagDetectorNodelet::updateNewlyIdentifiedTags(size_t tagIndex) {
        int currentId = mImmediateIds[tagIndex];

        //Create new struct for each tag
        LongRangeTagStruct lrt = createLrt(currentId, mImmediateCorners[tagIndex]);

        //Check if the tag was already detected and update hitCount to reflect
        if (mTags.contains(currentId)) {
            //Key exist sin mTags
            lrt.hitCount = std::min(mTags[currentId].hitCount + mTagIncrementWeight, mMaxHitCount);
        }
        mTags[currentId] = lrt;
    }

    cv::Point2f LongRangeTagDetectorNodelet::getTagCenterPixels(std::vector<cv::Point2f>& tagCorners) {
        cv::Point2f centerPoint;
        float centerX = 0;
        float centerY = 0;

        for (size_t i = 0; i < 4; i++) {
            centerX += tagCorners[i].x;
            centerY += tagCorners[i].y;
        }

        centerX /= 4.0;
        centerY /= 4.0;

        centerPoint.x = centerX;
        centerPoint.y = centerY;

        return centerPoint;
    }

    cv::Point2f LongRangeTagDetectorNodelet::getTagCenterOffsetPixels(std::vector<cv::Point2f>& tagCorners) const {
        cv::Point2f centerPoint = getTagCenterPixels(tagCorners);

        centerPoint.x -= static_cast<float>(mImgMsg.width);

        //-1 is necessary because of 0,0 being in the top left
        centerPoint.y = static_cast<float>(-1.0) * (centerPoint.y - static_cast<float>(mImgMsg.height));

        return centerPoint;
    }

    cv::Point2f LongRangeTagDetectorNodelet::getNormedTagCenterOffset(std::vector<cv::Point2f>& tagCorners) const {
        cv::Point2f offsetCenterPoint = getTagCenterOffsetPixels(tagCorners);

        offsetCenterPoint.x /= static_cast<float>(mImgMsg.width);
        offsetCenterPoint.y /= static_cast<float>(mImgMsg.height);

        return offsetCenterPoint;
    }

    float LongRangeTagDetectorNodelet::getTagBearing(cv::Point2f& tagCenter) const {
        //for HD720 resolution
        float bearing = -1 * ((float) tagCenter.x + 0.5) * mLongRangeFov;
        return bearing;
    }

    void LongRangeTagDetectorNodelet::publishPermanentTags() {
        //Loop through all the tagsj
        LongRangeTags tagsMessage; //

        for (auto& tag: mTags) {
            if (tag.second.hitCount >= mMinHitCountBeforePublish) {
                //LongRangeTag message
                LongRangeTag newTagMessage;

                //Fill in fields
                newTagMessage.id = tag.second.id;
                newTagMessage.bearing = getTagBearing(tag.second.imageCenter);
                //Add to back of tagsMessage
                tagsMessage.longRangeTags.push_back(newTagMessage);
            }
        }

        //tagsMessage should be a vector of LongRangeTag messages
        //Need something like an mTagsPublisher
        mLongRangeTagsPub.publish(tagsMessage);
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