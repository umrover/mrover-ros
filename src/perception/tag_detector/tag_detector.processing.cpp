#include "tag_detector.hpp"

#include <cmath>
#include <numeric>

constexpr size_t IMAGE_WIDTH_WARN_SIZE = 640;
constexpr size_t IMAGE_HEIGHT_WARN_SIZE = 480;

struct Point {
    float x, y, z;
    uint8_t r, g, b, a;
    float normal_x, normal_y, normal_z;
    float curvature;
} __attribute__((packed));

/**
 * @brief       Retrieve the pose of the tag in camera space
 * @param msg   3D Point Cloud with points stored relative to the camera
 * @param u     X Pixel Position
 * @param v     Y Pixel Position
 */
std::optional<SE3> getFidInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v) {
    if (u >= cloudPtr->width || v >= cloudPtr->height) {
        ROS_WARN("Tag center out of bounds: [%zu %zu]", u, v);
        return std::nullopt;
    }

    Point point = reinterpret_cast<Point const*>(cloudPtr->data.data())[u + v * cloudPtr->width];

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        ROS_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);
        return std::nullopt;
    }

    return {R3{point.x, point.y, point.z}};
}

/**
 * For each tag we have detected so far, fuse point cloud information.
 * This information is where it is in the world.
 *
 * @param msg   Point cloud message
 */
void TagDetectorNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    if (!mEnableDetections) return;

    ROS_DEBUG("Got point cloud %d", msg->header.seq);

    if (static_cast<int>(msg->height) != mImg.rows || static_cast<int>(msg->width) != mImg.cols) {
        ROS_INFO("Image size changed from [%d %d] to [%u %u]", mImg.cols, mImg.rows, msg->width, msg->height);
        mImg = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0}};
    }

    for (size_t i = 0; i < msg->height * msg->width; ++i) {
        Point const* point = reinterpret_cast<Point const*>(msg->data.data()) + i;
        mImg.at<cv::Vec3b>(static_cast<int>(i)) = {point->r, point->g, point->b};
    }

    // Detect the tag vertices in screen space and their respective ids
    // {mCorners, mIds} are the outputs from OpenCV
    cv::aruco::detectMarkers(mImg, mDictionary, mCorners, mIds, mDetectorParams);
    ROS_DEBUG("OpenCV detect size: %zu", mIds.size());

    // Update ID, image center, and increment hit count for all detected tags
    for (size_t i = 0; i < mIds.size(); ++i) {
        int id = mIds[i];
        Tag& tag = mTags[id];
        tag.hitCount = std::clamp(tag.hitCount + 1, 0, mMaxHitCount);
        tag.id = id;
        tag.imageCenter = std::accumulate(mCorners[i].begin(), mCorners[i].end(), cv::Point2f{}) / 4.0f;
        tag.tagInCam = getFidInCamFromPixel(msg, std::lround(tag.imageCenter.x), std::lround(tag.imageCenter.y));

        if (tag.tagInCam) {
            // Publish tag to immediate
            std::string immediateFrameId = "immediateFiducial" + std::to_string(tag.id);
            SE3::pushToTfTree(mTfBroadcaster, immediateFrameId, mBaseLinkFrameId, tag.tagInCam.value());
        }
    }

    // Handle tags that were not seen this update
    // Decrement their hit count and remove if they hit zero
    //    auto it = mTags.begin();
    //    while (it != mTags.end()) {
    //        auto& [id, tag] = *it;
    //        if (std::find(mIds.begin(), mIds.end(), id) == mIds.end()) {
    //            tag.hitCount--;
    //            if (tag.hitCount <= 0) {
    //                it = mTags.erase(it);
    //                continue;
    //            }
    //        }
    //        ++it;
    //    }

    //    // Publish all tags to the tf tree that have been seen enough times
    //    for (auto const& [id, tag]: mTags) {
    //        if (tag.hitCount >= mMinHitCountBeforePublish) {
    //            if (tag.tagInCam) {
    //                try {
    //                    std::string immediateFrameId = "immediateFiducial" + std::to_string(tag.id);
    //                    // Publish tag to odom
    //                    std::string const& parentFrameId = mUseOdom ? mOdomFrameId : mMapFrameId;
    //                    SE3 tagInParent = SE3::fromTfTree(mTfBuffer, parentFrameId, immediateFrameId);
    //                    SE3::pushToTfTree(mTfBroadcaster, "fiducial" + std::to_string(id), parentFrameId, tagInParent);
    //                } catch (tf2::ExtrapolationException const&) {
    //                    ROS_WARN("Old data for immediate tag");
    //                } catch (tf2::LookupException const&) {
    //                    ROS_WARN("Expected transform for immediate tag");
    //                }
    //            } else {
    //                ROS_DEBUG("Had tag detection but no corresponding point cloud information");
    //            }
    //        }
    //    }

    if (mPublishImages && mImgPub.getNumSubscribers()) {
        cv::aruco::drawDetectedMarkers(mImg, mCorners, mIds);
        mImgMsg.header.frame_id = "zed2i_left_camera_frame";
        mImgMsg.header.stamp = ros::Time::now();
        mImgMsg.height = mImg.rows;
        mImgMsg.width = mImg.cols;
        mImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
        mImgMsg.step = mImg.step;
        mImgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        size_t size = mImgMsg.step * mImgMsg.height;
        mImgMsg.data.resize(size);
        std::memcpy(mImgMsg.data.data(), mImg.data, size);
        mImgPub.publish(mImgMsg);
    }

    size_t detectedCount = mIds.size();
    if (!mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value()) {
        mPrevDetectedCount = detectedCount;
        ROS_INFO("Detected %zu markers", detectedCount);
    }

    mSeqNum++;
}
