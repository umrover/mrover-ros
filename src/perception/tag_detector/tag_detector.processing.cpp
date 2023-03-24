#include "tag_detector.hpp"

#include "filter.hpp"
#include <cmath>

constexpr size_t IMAGE_WIDTH_WARN_SIZE = 640;
constexpr size_t IMAGE_HEIGHT_WARN_SIZE = 480;

/**
 * Detect tags from raw image using OpenCV and calculate their screen space centers.
 * Tag pose information relative to the camera in 3D space is filled in when we receive point cloud data.
 *
 * @param msg
 */
// void TagDetectorNode::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
    
//     if (!mEnableDetections) return;

//     bool isInSim = false;
//     mNh.getParam("use_sim_time", isInSim);

//     if (!isInSim && (msg->width <= IMAGE_WIDTH_WARN_SIZE || msg->height <= IMAGE_HEIGHT_WARN_SIZE)) {
//         ROS_WARN("Input image is below 640x480 resolution. Tag detection may be poor");
//     }

//     ROS_DEBUG("Got image %d", msg->header.seq);

//     try {
//         cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

//         // Detect the tag vertices in screen space and their respective ids
//         // {mCorners, mIds} are the outputs from OpenCV
//         cv::aruco::detectMarkers(imagePtr->image, mDictionary, mCorners, mIds, mDetectorParams);

//         ROS_DEBUG("OpenCV detect size: %zu", mIds.size());

//         // Update ID, image center, and increment hit count for all detected tags
//         for (size_t i = 0; i < mIds.size(); ++i) {
//             int id = mIds[i];
//             Tag& tag = mTags[id];
//             tag.hitCount = std::clamp(tag.hitCount + 1, 0, mMaxHitCount);
//             tag.id = id;
//             tag.imageCenter = std::accumulate(mCorners[i].begin(), mCorners[i].end(), cv::Point2f{}) / 4.0f;

//             if (tag.tagInCam) {
//                 // Publish tag to immediate
//                 std::string immediateFrameId = "immediateFiducial" + std::to_string(tag.id);
//                 SE3::pushToTfTree(mTfBroadcaster, immediateFrameId, mBaseLinkFrameId, tag.tagInCam.value());
//             }
//         }

//         // Handle tags that were not seen this update
//         // Decrement their hit count and remove if they hit zero
//         auto it = mTags.begin();
//         while (it != mTags.end()) {
//             auto& [id, tag] = *it;
//             if (std::find(mIds.begin(), mIds.end(), id) == mIds.end()) {
//                 tag.hitCount--;
//                 if (tag.hitCount <= 0) {
//                     it = mTags.erase(it);
//                 }
//             } else {
//                 ++it;
//             }
//         }

//         // Publish all tags to the tf tree that have been seen enough times
//         for (auto const& [id, tag]: mTags) {
//             if (tag.hitCount >= mMinHitCountBeforePublish) {
//                 if (tag.tagInCam) {
//                     try {
//                         std::string immediateFrameId = "immediateFiducial" + std::to_string(tag.id);
//                         // Publish tag to odom
//                         std::string const& frameId = mUseOdom ? mOdomFrameId : mMapFrameId;
//                         SE3 tagInOdom = SE3::fromTfTree(mTfBuffer, frameId, immediateFrameId);
//                         SE3::pushToTfTree(mTfBroadcaster, "fiducial" + std::to_string(id), frameId, tagInOdom);
//                     } catch (tf2::ExtrapolationException const&) {
//                         ROS_WARN("Old data for immediate tag");
//                     } catch (tf2::LookupException const&) {
//                         ROS_WARN("Expected transform for immediate tag");
//                     }
//                 } else {
//                     ROS_DEBUG("Had tag detection but no corresponding point cloud information");
//                 }
//             }
//         }

//         size_t detectedCount = mIds.size();
//         if (!mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value()) {
//             mPrevDetectedCount = detectedCount;
//             ROS_DEBUG("Detected %zu markers", detectedCount);
//         }

//         if (!mTags.empty()) {
//             cv::aruco::drawDetectedMarkers(imagePtr->image, mCorners, mIds);
//         }

//         if (mPublishImages) {
//             mImgPub.publish(imagePtr->toImageMsg());
//         }

//         mSeqNum++;
//     } catch (cv_bridge::Exception const& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     } catch (cv::Exception const& e) {
//         ROS_ERROR("cv exception: %s", e.what());
//     }
// }

/**
 * @brief       Retrieve the pose of the tag in camera space
 * @param msg   3D Point Cloud with points stored relative to the camera
 * @param u     X Pixel Position
 * @param v     Y Pixel Position
 */
std::optional<SE3> getFidInCamFromPixel(PointCloudPtr const& cloudPtr, size_t u, size_t v) {
    if (u >= cloudPtr->width || v >= cloudPtr->height) {
        ROS_WARN("Tag center out of bounds");
        return std::nullopt;
    }
    
    pcl::PointXYZRGBNormal const& point = cloudPtr->at(static_cast<int>(u), static_cast<int>(v));
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        ROS_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);
        return std::nullopt;
    }

    return SE3{R3{point.x, point.y, point.z}};
}

/**
 * For each tag we have detected so far, fuse point cloud information.
 * This information is where it is in the world.
 *
 * @param msg   Point cloud message
 */
void TagDetectorNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    if (!mEnableDetections) return;

    pcl::fromROSMsg(*msg, *mCloudPtr);
    if (mCloudPtr->empty()) {
        ROS_WARN("Empty point cloud received");
        return;
    }

    for (auto& [id, tag]: mTags) {
        size_t u = std::lround(tag.imageCenter.x);
        size_t v = std::lround(tag.imageCenter.y);

        try {
            tag.tagInCam = getFidInCamFromPixel(mCloudPtr, u, v);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform lookup error: %s", ex.what());
        }
    }
}
