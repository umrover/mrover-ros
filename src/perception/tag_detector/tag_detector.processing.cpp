#include "tag_detector.hpp"

#include "point.hpp"

namespace mrover {

    /**
     * For each tag we have detected so far, fuse point cloud information.
     * This information is where it is in the world.
     *
     * @param msg   Point cloud message
     */
    auto StereoTagDetectorNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mEnableDetections) return;

        // OpenCV needs a dense BGR image |BGR|...| but out point cloud is
        // |BGRAXYZ...|...| So we need to copy the data into the correct format
        if (static_cast<int>(msg->height) != mBgrImage.rows || static_cast<int>(msg->width) != mBgrImage.cols) {
            NODELET_INFO_STREAM(std::format("Image size changed from [{}, {}] to [{}, {}]", mBgrImage.cols, mBgrImage.rows, msg->width, msg->height));
            mBgrImage = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0}};
        }
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(mBgrImage.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + mBgrImage.total(), [&](cv::Vec3b& pixel) {
            std::size_t i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
        });
        mProfiler.measureEvent("Conversion");

        // Detect the tag vertices in screen space and their respective ids
        // {mImmediateCorneres, mImmediateIds} are the outputs from OpenCV
        cv::aruco::detectMarkers(mBgrImage, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);
        mProfiler.measureEvent("Detection");

        publishThresholdedImage();
        mProfiler.measureEvent("Threshold");

        // Update ID, image center, and increment hit count for all detected tags
        for (std::size_t i = 0; i < mImmediateIds.size(); ++i) {
            int id = mImmediateIds[i];
            Tag& tag = mTags[id];
            tag.hitCount = std::clamp(tag.hitCount + mTagIncrementWeight, 0, mMaxHitCount);
            tag.id = id;
            tag.imageCenter = std::reduce(mImmediateCorners[i].begin(), mImmediateCorners[i].end()) / static_cast<float>(mImmediateCorners[i].size());
            tag.tagInCam = getTagInCamFromPixel(msg, std::lround(tag.imageCenter.x), std::lround(tag.imageCenter.y));

            if (!tag.tagInCam) continue;

            // Publish tag to immediate
            std::string immediateFrameId = std::format("immediateTag{}", tag.id);
            SE3Conversions::pushToTfTree(mTfBroadcaster, immediateFrameId, mCameraFrameId, tag.tagInCam.value());
        }
        // Handle tags that were not seen this update
        // Decrement their hit count and remove if they hit zero
        auto it = mTags.begin();
        while (it != mTags.end()) {
            if (auto& [id, tag] = *it; std::ranges::find(mImmediateIds, id) == mImmediateIds.end()) {
                tag.hitCount -= mTagDecrementWeight;
                tag.tagInCam = std::nullopt;
                if (tag.hitCount <= 0) {
                    it = mTags.erase(it);
                    continue;
                }
            }
            ++it;
        }
        // Publish all tags to the tf tree that have been seen enough times
        for (auto const& [id, tag]: mTags) {
            if (tag.hitCount >= mMinHitCountBeforePublish && tag.tagInCam) {
                try {
                    // Use the TF tree to transform the tag from the camera frame to the map frame
                    // Then publish it in the map frame persistently
                    std::string immediateFrameId = std::format("immediateTag{}", tag.id);
                    SE3d tagInParent = SE3Conversions::fromTfTree(mTfBuffer, immediateFrameId, mMapFrameId);
                    SE3Conversions::pushToTfTree(mTfBroadcaster, std::format("tag{}", tag.id), mMapFrameId, tagInParent);
                } catch (tf2::ExtrapolationException const&) {
                    NODELET_WARN("Old data for immediate tag");
                } catch (tf2::LookupException const&) {
                    NODELET_WARN("Expected transform for immediate tag");
                } catch (tf2::ConnectivityException const&) {
                    NODELET_WARN("Expected connection to odom frame. Is visual odometry running?");
                }
            }
        }
        publishDetectedTags();
        mProfiler.measureEvent("Publication");
    }

    auto ImageTagDetectorNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        assert(msg->encoding == sensor_msgs::image_encodings::BGRA8);

        if (!mEnableDetections) return;

        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())}; // No copy is made, it simply wraps the pointer
        cv::cvtColor(bgraImage, mBgrImage, cv::COLOR_BGRA2BGR);
        mProfiler.measureEvent("Conversion");

        // Detect the tag vertices in screen space and their respective ids
        // {mImmediateCorneres, mImmediateIds} are the outputs from OpenCV
        cv::aruco::detectMarkers(mBgrImage, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);
        mProfiler.measureEvent("Detection");

        ImageTargets targets;
        for (std::size_t i = 0; i < mImmediateIds.size(); ++i) {
            ImageTarget newTarget;
            newTarget.name = std::format("tag{}", mImmediateIds[i]);
            newTarget.bearing = getTagBearing(mBgrImage, mImmediateCorners[i]);
            targets.targets.push_back(newTarget);
        }
        mTargetsPub.publish(targets);
        publishDetectedTags();
        mProfiler.measureEvent("Publication");
    }

    auto TagDetectorNodeletBase::publishDetectedTags() -> void {
        if (mDetectedImagePub.getNumSubscribers()) {
            cv::aruco::drawDetectedMarkers(mBgrImage, mImmediateCorners, mImmediateIds);
            // Max number of tags the hit counter can display = 10;
            if (!mTags.empty()) {
                // TODO: remove some magic numbers in this block
                int tagCount = 1;
                auto tagBoxWidth = static_cast<int>(mBgrImage.cols / (mTags.size() * 2));
                for (auto& [id, tag]: mTags) {
                    cv::Scalar color{255, 0, 0};
                    cv::Point pt{tagBoxWidth * tagCount, mBgrImage.rows / 10};
                    std::string text = std::format("id{}:{}", id, tag.hitCount);
                    cv::putText(mBgrImage, text, pt, cv::FONT_HERSHEY_COMPLEX, mBgrImage.cols / 800.0, color, mBgrImage.cols / 300);
                    ++tagCount;
                }
            }
            mDetectionsImageMessage.header.stamp = ros::Time::now();
            mDetectionsImageMessage.header.frame_id = "zed_left_camera_frame";
            mDetectionsImageMessage.height = mBgrImage.rows;
            mDetectionsImageMessage.width = mBgrImage.cols;
            mDetectionsImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;
            mDetectionsImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            mDetectionsImageMessage.step = 4 * mDetectionsImageMessage.width;
            mDetectionsImageMessage.data.resize(mDetectionsImageMessage.step * mDetectionsImageMessage.height);
            cv::Mat imageMessageWrapper{mBgrImage.rows, mBgrImage.cols, CV_8UC4, mDetectionsImageMessage.data.data()};
            cv::cvtColor(mBgrImage, imageMessageWrapper, cv::COLOR_BGR2BGRA);

            mDetectedImagePub.publish(mDetectionsImageMessage);
        }

        std::size_t detectedCount = mImmediateIds.size();
        NODELET_INFO_COND(!mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value(), "Detected %zu markers", detectedCount);
        mPrevDetectedCount = detectedCount;
    }

    /**
     * @brief           Retrieve the pose of the tag in camera space
     * @param cloudPtr  3D Point Cloud with points stored relative to the camera
     * @param u         X Pixel Position
     * @param v         Y Pixel Position
     */
    auto StereoTagDetectorNodelet::getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, std::size_t u, std::size_t v) const -> std::optional<SE3d> {
        assert(cloudPtr);

        if (u >= cloudPtr->width || v >= cloudPtr->height) {
            NODELET_WARN("Tag center out of bounds: [%zu %zu]", u, v);
            return std::nullopt;
        }

        Point const& point = reinterpret_cast<Point const*>(cloudPtr->data.data())[u + v * cloudPtr->width];

        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            NODELET_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);
            return std::nullopt;
        }

        return std::make_optional<SE3d>(R3{point.x, point.y, point.z}, SO3d::Identity());
    }

    auto ImageTagDetectorNodelet::getTagBearing(cv::InputArray image, std::span<cv::Point2f const> tagCorners) const -> float {
        // Takes the average of the corners
        cv::Point2f center = std::reduce(tagCorners.begin(), tagCorners.end()) / static_cast<float>(tagCorners.size());
        float xNormalized = center.x / static_cast<float>(image.cols());
        float xRecentered = 0.5f - xNormalized;
        float bearingDegrees = xRecentered * mCameraHorizontalFov;
        return bearingDegrees * std::numbers::pi_v<float> / 180.0f;
    }

} // namespace mrover
