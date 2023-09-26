#include "long_range_tag_detector.hpp"

#include "../point.hpp"
#include <opencv2/core/types.hpp>

namespace mrover {

    /**
     * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
     *
     * @param msg   image message
     */
    void LongRangeTagDetectorNodelet::imageCallback(sensor_msgs::Image const& msg) {
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
            // LongRangeTagDetectorNodelet::publishDrawnImages()
        }

        //PUBLISH TAGS
    }

    //HELPER FUNCTIONS

    void LongRangeTagDetectorNodelet::updateImageMatrices(sensor_msgs::Image const& msg) {
        //Store image message to mImgMsg member variable
        mImgMsg = msg;

        cv::Mat cvImage{static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3, const_cast<uint8_t*>(msg.data.data())};

        //Store to mImg member variable - unsure if necessary but need it to persist
        cvImage.copyTo(mImg);

        // TODO: Set the grayImage if neccesary
    }

    void LongRangeTagDetectorNodelet::runTagDetection() {
        cv::aruco::detectMarkers(mImg, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);
    }

    void LongRangeTagDetectorNodelet::updateHitCounts() {
        //loop through all identified IDs
        for (size_t i = 0; i < mImmediateIds.size(); i++) {
            int currentId = mImmediateIds[i];

            //Create new struct for each tag
            LongRangeTag lrt = createLrt(currentId, mImmediateCorners[i]);

            //Check if the tag was already detected and update hitCount to reflect
            if (mTags.contains(currentId)) {
                //Key exist sin mTags
                lrt.hitCount = std::min(mTags[currentId].hitCount + mTagIncrementWeight, mMaxHitCount);
            }

            mTags[currentId] = lrt;
        }

        //Now decrement all the hitcounts for tags that were not updated
        // Set updated status to false

        for (auto& mTag: mTags) {
            LongRangeTag& curtag = mTag.second;

            if (curtag.updated) {
                curtag.updated = false;
            } else {
                //Decrement weight of undetected tags
                curtag.hitCount -= mTagDecrementWeight;

                //if the value has fallen belown the minimum, remove it
                if (curtag.hitCount <= mTagRemoveWeight) {
                    mTags.erase(mTag.first);
                }
            }
        }


        //decrement non updated & set updated status to false
    }

    LongRangeTag LongRangeTagDetectorNodelet::createLrt(int tagId, std::vector<cv::Point2f>& tagCorners) {
        LongRangeTag lrt;

        lrt.hitCount = mBaseHitCount; //Start at base hit count value
        lrt.id = tagId;
        lrt.updated = true;

        lrt.imageCenter = getImageCenter(tagCorners);

        return lrt;
    }

} // namespace mrover