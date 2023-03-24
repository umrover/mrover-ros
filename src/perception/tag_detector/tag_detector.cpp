#include "tag_detector.hpp"

void TagDetectorNode::configCallback(mrover::DetectorParamsConfig& config, uint32_t level) {
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

bool TagDetectorNode::enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    mEnableDetections = req.data;
    if (mEnableDetections) {
        res.message = "Enabled tag detections.";
        ROS_INFO("Enabled tag detections.");
    } else {
        res.message = "Disabled tag detections.";
        ROS_INFO("Disabled tag detections.");
    }

    res.success = true;
    return true;
}

TagDetectorNode::TagDetectorNode(ros::NodeHandle const& nh, ros::NodeHandle const& pnh) : mNh{nh}, mPnh{pnh}, mIt{mNh}, mTfListener{mTfBuffer} {
    mDetectorParams = new cv::aruco::DetectorParameters();
    auto defaultDetectorParams = cv::aruco::DetectorParameters::create();
    int dictionaryNumber;

    mNh.param<bool>("use_odom_frame", mUseOdom, false);
    mNh.param<std::string>("odom_frame", mOdomFrameId, "odom");
    mNh.param<std::string>("map_frame", mMapFrameId, "map");
    mNh.param<std::string>("rover_frame", mCameraFrameId, "zed2i_left_camera_frame");

    mPnh.param<bool>("publish_images", mPublishImages, true);
    mPnh.param<int>("dictionary", dictionaryNumber, 0);

    mImgPub = mIt.advertise("tag_detection", 1);
    mDictionary = cv::aruco::getPredefinedDictionary(dictionaryNumber);

    mPcSub = mNh.subscribe("camera/left/points", 1, &TagDetectorNode::pointCloudCallback, this);
    mServiceEnableDetections = mNh.advertiseService("enable_detections", &TagDetectorNode::enableDetectionsCallback, this);

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

    ROS_INFO("Tag detection ready");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_detector");

    [[maybe_unused]] auto node = std::make_unique<TagDetectorNode>();

    ros::spin();

    return EXIT_SUCCESS;
}
