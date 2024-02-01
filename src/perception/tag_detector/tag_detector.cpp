#include "tag_detector.hpp"

namespace mrover {

    void TagDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
        auto defaultDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
        int dictionaryNumber;

        mNh.param<bool>("use_odom_frame", mUseOdom, false);
        mNh.param<std::string>("odom_frame", mOdomFrameId, "odom");
        mNh.param<std::string>("world_frame", mMapFrameId, "map");
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed2i_left_camera_frame");

        mPnh.param<bool>("publish_images", mPublishImages, true);
        mPnh.param<int>("dictionary", dictionaryNumber, static_cast<int>(cv::aruco::DICT_4X4_50));
        mPnh.param<int>("min_hit_count_before_publish", mMinHitCountBeforePublish, 5);
        mPnh.param<int>("max_hit_count", mMaxHitCount, 5);
        mPnh.param<int>("tag_increment_weight", mTagIncrementWeight, 2);
        mPnh.param<int>("tag_decrement_weight", mTagDecrementWeight, 1);

        mImgPub = mNh.advertise<sensor_msgs::Image>("tag_detection", 1);
        mDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dictionaryNumber));

        mPcSub = mNh.subscribe("camera/left/points", 1, &TagDetectorNodelet::pointCloudCallback, this);
        mServiceEnableDetections = mNh.advertiseService("enable_detections", &TagDetectorNodelet::enableDetectionsCallback, this);

        // Lambda handles passing class pointer (implicit first parameter) to configCallback
        mCallbackType = [this](mrover::DetectorParamsConfig& config, uint32_t level) { configCallback(config, level); };
        mConfigServer.setCallback(mCallbackType);

        mPnh.param<double>("adaptiveThreshConstant",
                           mDetectorParams->adaptiveThreshConstant, defaultDetectorParams->adaptiveThreshConstant);
        mPnh.param<int>("adaptiveThreshWinSizeMax",
                        mDetectorParams->adaptiveThreshWinSizeMax, defaultDetectorParams->adaptiveThreshWinSizeMax);
        mPnh.param<int>("adaptiveThreshWinSizeMin",
                        mDetectorParams->adaptiveThreshWinSizeMin, defaultDetectorParams->adaptiveThreshWinSizeMin);
        mPnh.param<int>("adaptiveThreshWinSizeStep",
                        mDetectorParams->adaptiveThreshWinSizeStep, defaultDetectorParams->adaptiveThreshWinSizeStep);
        mPnh.param<int>("cornerRefinementMaxIterations",
                        mDetectorParams->cornerRefinementMaxIterations,
                        defaultDetectorParams->cornerRefinementMaxIterations);
        mPnh.param<double>("cornerRefinementMinAccuracy",
                           mDetectorParams->cornerRefinementMinAccuracy,
                           defaultDetectorParams->cornerRefinementMinAccuracy);
        mPnh.param<int>("cornerRefinementWinSize",
                        mDetectorParams->cornerRefinementWinSize, defaultDetectorParams->cornerRefinementWinSize);

        bool doCornerRefinement = true;
        mPnh.param<bool>("doCornerRefinement", doCornerRefinement, true);
        if (doCornerRefinement) {
            bool cornerRefinementSubPix = true;
            mPnh.param<bool>("cornerRefinementSubPix", cornerRefinementSubPix, true);
            if (cornerRefinementSubPix) {
                mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            } else {
                mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
            }
        } else {
            mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
        }

        mPnh.param<double>("errorCorrectionRate",
                           mDetectorParams->errorCorrectionRate, defaultDetectorParams->errorCorrectionRate);
        mPnh.param<double>("minCornerDistanceRate",
                           mDetectorParams->minCornerDistanceRate, defaultDetectorParams->minCornerDistanceRate);
        mPnh.param<int>("markerBorderBits",
                        mDetectorParams->markerBorderBits, defaultDetectorParams->markerBorderBits);
        mPnh.param<double>("maxErroneousBitsInBorderRate",
                           mDetectorParams->maxErroneousBitsInBorderRate,
                           defaultDetectorParams->maxErroneousBitsInBorderRate);
        mPnh.param<int>("minDistanceToBorder",
                        mDetectorParams->minDistanceToBorder, defaultDetectorParams->minDistanceToBorder);
        mPnh.param<double>("minMarkerDistanceRate",
                           mDetectorParams->minMarkerDistanceRate, defaultDetectorParams->minMarkerDistanceRate);
        mPnh.param<double>("minMarkerPerimeterRate",
                           mDetectorParams->minMarkerPerimeterRate, defaultDetectorParams->minMarkerPerimeterRate);
        mPnh.param<double>("maxMarkerPerimeterRate",
                           mDetectorParams->maxMarkerPerimeterRate, defaultDetectorParams->maxMarkerPerimeterRate);
        mPnh.param<double>("minOtsuStdDev", mDetectorParams->minOtsuStdDev, defaultDetectorParams->minOtsuStdDev);
        mPnh.param<double>("perspectiveRemoveIgnoredMarginPerCell",
                           mDetectorParams->perspectiveRemoveIgnoredMarginPerCell,
                           defaultDetectorParams->perspectiveRemoveIgnoredMarginPerCell);
        mPnh.param<int>("perspectiveRemovePixelPerCell",
                        mDetectorParams->perspectiveRemovePixelPerCell,
                        defaultDetectorParams->perspectiveRemovePixelPerCell);
        mPnh.param<double>("polygonalApproxAccuracyRate",
                           mDetectorParams->polygonalApproxAccuracyRate,
                           defaultDetectorParams->polygonalApproxAccuracyRate);

        NODELET_INFO("Tag detection ready, use odom frame: %s, min hit count: %d, max hit count: %d, hit increment weight: %d, hit decrement weight: %d", mUseOdom ? "true" : "false", mMinHitCountBeforePublish, mMaxHitCount, mTagIncrementWeight, mTagDecrementWeight);
    }

    void TagDetectorNodelet::configCallback(mrover::DetectorParamsConfig& config, uint32_t level) {
        // Don't load initial config, since it will overwrite the rosparam settings
        if (level == std::numeric_limits<uint32_t>::max()) return;

        mDetectorParams->adaptiveThreshConstant = config.adaptiveThreshConstant;
        mDetectorParams->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
        mDetectorParams->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
        mDetectorParams->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
        mDetectorParams->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
        mDetectorParams->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
        mDetectorParams->cornerRefinementWinSize = config.cornerRefinementWinSize;
        if (config.doCornerRefinement) {
            if (config.cornerRefinementSubpix) {
                mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            } else {
                mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
            }
        } else {
            mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
        }
        mDetectorParams->errorCorrectionRate = config.errorCorrectionRate;
        mDetectorParams->minCornerDistanceRate = config.minCornerDistanceRate;
        mDetectorParams->markerBorderBits = config.markerBorderBits;
        mDetectorParams->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
        mDetectorParams->minDistanceToBorder = config.minDistanceToBorder;
        mDetectorParams->minMarkerDistanceRate = config.minMarkerDistanceRate;
        mDetectorParams->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
        mDetectorParams->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
        mDetectorParams->minOtsuStdDev = config.minOtsuStdDev;
        mDetectorParams->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
        mDetectorParams->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
        mDetectorParams->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
    }

    bool TagDetectorNodelet::enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        mEnableDetections = req.data;
        if (mEnableDetections) {
            res.message = "Enabled tag detections.";
            NODELET_INFO("Enabled tag detections.");
        } else {
            res.message = "Disabled tag detections.";
            NODELET_INFO("Disabled tag detections.");
        }

        res.success = true;
        return true;
    }

    // SE3d TagDetectorNodelet::SE3fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) {
    //     geometry_msgs::TransformStamped transform = buffer.lookupTransform(fromFrameId, toFrameId, ros::Time{});
    //     return SE3fromTf(transform.transform);
    // }

    // void TagDetectorNodelet::pushSE3ToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf) {
    //     broadcaster.sendTransform(SE3toTransformStamped(tf, parentFrameId, childFrameId));
    // }

    // SE3d TagDetectorNodelet::SE3fromTf(geometry_msgs::Transform const& transform) {
    //     return {{transform.translation.x, transform.translation.y, transform.translation.z},
    //             Eigen::Quaterniond{transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z}};
    // }

    // SE3d TagDetectorNodelet::SE3fromPose(geometry_msgs::Pose const& pose) {
    //     return {{pose.position.x, pose.position.y, pose.position.z},
    //             Eigen::Quaterniond{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}};
    // }

    // geometry_msgs::Pose TagDetectorNodelet::SE3toPose(SE3d const& tf) {
    //     geometry_msgs::Pose pose;
    //     pose.position.x = tf.x();
    //     pose.position.y = tf.y();
    //     pose.position.z = tf.z();
    //     pose.orientation.x = tf.quat().x();
    //     pose.orientation.y = tf.quat().y();
    //     pose.orientation.z = tf.quat().z();
    //     pose.orientation.w = tf.quat().w();
    //     return pose;
    // }

    // geometry_msgs::Transform TagDetectorNodelet::SE3toTransform(SE3d const& tf) {
    //     geometry_msgs::Transform transform;
    //     transform.translation.x = tf.x();
    //     transform.translation.y = tf.y();
    //     transform.translation.z = tf.z();
    //     transform.rotation.x = tf.quat().x();
    //     transform.rotation.y = tf.quat().y();
    //     transform.rotation.z = tf.quat().z();
    //     transform.rotation.w = tf.quat().w();
    //     return transform;
    // }

    // geometry_msgs::PoseStamped TagDetectorNodelet::SE3toPoseStamped(SE3d const& tf, std::string const& frameId) {
    //     geometry_msgs::PoseStamped pose;
    //     pose.pose = SE3toPose(tf);
    //     pose.header.frame_id = frameId;
    //     pose.header.stamp = ros::Time::now();
    //     return pose;
    // }

    // geometry_msgs::TransformStamped TagDetectorNodelet::SE3toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId) {
    //     geometry_msgs::TransformStamped transform;
    //     transform.transform = SE3toTransform(tf);
    //     transform.header.frame_id = parentFrameId;
    //     transform.header.stamp = ros::Time::now();
    //     transform.child_frame_id = childFrameId;
    //     return transform;
    // }


} // namespace mrover
