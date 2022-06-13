#include "aruco_detect.hpp"

#include "filter.hpp"

/**
 * Detect fiducials from raw image and publish all of them.
 *
 * @param msg
 */
void FiducialsNode::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
    if (!mEnableDetections) return;

    if (mIsVerbose) {
        ROS_INFO("Got image %d", msg->header.seq);
    }

    try {
        mCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::aruco::detectMarkers(mCvPtr->image, mDictionary, mCornersCache, mIdsCache, mDetectorParams);

        // Set IDs and centers of tags
        for (size_t i = 0; i < mIdsCache.size(); ++i) {
            int id = mIdsCache[i];
            cv::Point2f imageCenter = std::accumulate(mCornersCache[i].begin(), mCornersCache[i].end(), cv::Point2f{}) / 4.0f;
            mImmediateFiducials[id].id = id;
            mImmediateFiducials[id].imageCenter = imageCenter;
        }

        // Remove all centers from tags that are no longer in sight
        auto it = mImmediateFiducials.begin();
        while (it != mImmediateFiducials.end()) {
            if (std::find(mIdsCache.begin(), mIdsCache.end(), it->first) == mIdsCache.end()) {
                it = mImmediateFiducials.erase(it);
            } else {
                ++it;
            }
        }

        fiducial_msgs::FiducialTransformArray fidArray{};
        fidArray.header.frame_id = ODOM_FRAME;
        fidArray.header.stamp = ros::Time::now();
        fidArray.header.seq = mSeqNum;
        fidArray.transforms.reserve(mPersistentFiducials.size());

        // Add readings to the persistent representations of the fiducials
        for (auto [id, immediateFid]: mImmediateFiducials) {
            PersistentFiducial& fid = mPersistentFiducials[id];
            if (!immediateFid.fidInCam.has_value()) continue;

            fid.id = id;
            SE3 fidInOdom = SE3::transform(mTfBuffer, ROVER_FRAME, ODOM_FRAME, immediateFid.fidInCam.value(), mSeqNum);
            fid.setFilterParams(mFilterCount, mFilterProportion);
            fid.addReading(fidInOdom);
        }

        // Send all transforms of persistent fiducials
        for (auto [id, fid]: mPersistentFiducials) {
            if (!fid.fidInOdomX.ready()) continue;

            geometry_msgs::TransformStamped tf{};
            tf.child_frame_id = "fiducial" + std::to_string(id);
            tf.header.frame_id = ODOM_FRAME;
            tf.header.stamp = ros::Time::now();
            tf.header.seq = mSeqNum;
            tf.transform.translation.x = fid.fidInOdomX.get();
            tf.transform.translation.y = fid.fidInOdomY.get();
            tf.transform.translation.z = fid.fidInOdomZ.get();
            tf.transform.rotation.w = 1.0;
            mTfBroadcaster.sendTransform(tf);
            fiducial_msgs::FiducialTransform fidTf{};
            fidTf.transform = tf.transform;
            fidTf.fiducial_id = id;
            fidArray.transforms.push_back(fidTf);
        }

        mFidPub.publish(fidArray);

        size_t detectedCount = mIdsCache.size();
        if (mIsVerbose || !mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value()) {
            mPrevDetectedCount = detectedCount;
            ROS_INFO("Detected %zu markers", detectedCount);
        }

        if (!mImmediateFiducials.empty()) {
            cv::aruco::drawDetectedMarkers(mCvPtr->image, mCornersCache, mIdsCache);
        }

        if (mPublishImages) {
            mImgPub.publish(mCvPtr->toImageMsg());
        }

        mSeqNum++;
    } catch (cv_bridge::Exception const& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (cv::Exception const& e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

SE3 getCamToFidFromPixel(sensor_msgs::PointCloud2ConstPtr const& msg, size_t u, size_t v) {
    // Could be done using PCL camToPoint clouds instead
    size_t arrayPos = v * msg->row_step + u * msg->point_step;
    size_t arrayPosY = arrayPos + msg->fields[0].offset;
    size_t arrayPosZ = arrayPos + msg->fields[1].offset;
    size_t arrayPosX = arrayPos + msg->fields[2].offset;

    cv::Point3f point;
    std::memcpy(&point.x, &msg->data[arrayPosX], sizeof(point.x));
    std::memcpy(&point.y, &msg->data[arrayPosY], sizeof(point.y));
    std::memcpy(&point.z, &msg->data[arrayPosZ], sizeof(point.z));

    SE3 camToPoint{};
    camToPoint.orientation.w = 1.0;
    camToPoint.position.x = +point.x;
    camToPoint.position.y = -point.y;
    camToPoint.position.z = +point.z;
    return camToPoint;
}

/**
 * For each active fiducial image we have detected so far, fuse point cloud information.
 * This information is where it is in the world.
 *
 * @param msg   Point cloud message
 */
void FiducialsNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    for (auto& [id, fid]: mImmediateFiducials) {
        size_t u = std::lround(fid.imageCenter.x);
        size_t v = std::lround(fid.imageCenter.y);

        try {
            fid.fidInCam = getCamToFidFromPixel(msg, u, v);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform lookup error: %s", ex.what());
        }
    }
}
