#include "aruco_detect.hpp"

cv::Point3f getWorldPosFromPixel(sensor_msgs::PointCloud2ConstPtr const& msg, size_t u, size_t v) {
    // Could be done using PCL point clouds instead
    size_t arrayPos = v * msg->row_step + u * msg->point_step;
    size_t arrayPosX = arrayPos + msg->fields[0].offset;
    size_t arrayPosY = arrayPos + msg->fields[1].offset;
    size_t arrayPosZ = arrayPos + msg->fields[2].offset;

    cv::Point3f point;
    std::memcpy(&point.x, &msg->data[arrayPosX], sizeof(point.x));
    std::memcpy(&point.y, &msg->data[arrayPosY], sizeof(point.y));
    std::memcpy(&point.z, &msg->data[arrayPosZ], sizeof(point.z));
    return point;
}

void FiducialsNode::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
    if (!mEnableDetections) {
        return;
    }

    if (mIsVerbose) {
        ROS_INFO("Got image %d", msg->header.seq);
    }

    fiducial_msgs::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id = mFrameId;
    fva.image_seq = static_cast<int>(msg->header.seq);

    try {
        mCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::aruco::detectMarkers(mCvPtr->image, mDictionary, mCornersCache, mIdsCache, mDetectorParams);

        // Set IDs and centers of tags
        for (size_t i = 0; i < mIdsCache.size(); ++i) {
            int id = mIdsCache[i];
            cv::Point2f imageCenter = std::accumulate(mCornersCache[i].begin(), mCornersCache[i].end(), cv::Point2f{}) / 4.0f;
            mFiducials[id].id = id;
            mFiducials[id].imageCenter = imageCenter;
        }

        // Remove all centers from tags that are no longer in sight
        for (auto& [id, fid]: mFiducials) {
            if (std::find(mIdsCache.begin(), mIdsCache.end(), id) == mIdsCache.end()) {
                fid.imageCenter = std::nullopt;
            }
        }

        size_t detectedCount = mIdsCache.size();
        if (mIsVerbose || detectedCount != mPrevDetectedCount) {
            mPrevDetectedCount = detectedCount;
            ROS_INFO("Detected %zu markers", detectedCount);
        }

        for (size_t i = 0; i < mIdsCache.size(); i++) {
            fiducial_msgs::Fiducial fid;
            fid.fiducial_id = mIdsCache[i];
            fid.x0 = mCornersCache[i][0].x;
            fid.y0 = mCornersCache[i][0].y;
            fid.x1 = mCornersCache[i][1].x;
            fid.y1 = mCornersCache[i][1].y;
            fid.x2 = mCornersCache[i][2].x;
            fid.y2 = mCornersCache[i][2].y;
            fid.x3 = mCornersCache[i][3].x;
            fid.y3 = mCornersCache[i][3].y;
            fva.fiducials.push_back(fid);
        }

        mVerticesPub.publish(fva);

        if (!mFiducials.empty()) {
            cv::aruco::drawDetectedMarkers(mCvPtr->image, mCornersCache, mIdsCache);
        }

        if (mPublishImages) {
            mImgPub.publish(mCvPtr->toImageMsg());
        }
    } catch (cv_bridge::Exception const& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (cv::Exception const& e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

void FiducialsNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    for (auto& [id, fid]: mFiducials) {
        if (!fid.imageCenter) continue;

        size_t u = std::lround(fid.imageCenter->x);
        size_t v = std::lround(fid.imageCenter->y);
        fid.worldPosition = getWorldPosFromPixel(msg, u, v);
    }
}
