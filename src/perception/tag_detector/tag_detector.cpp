#include "tag_detector.hpp"

namespace mrover {

    auto TagDetectorNodeletBase::onInit() -> void {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
        auto defaultDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();

        mNh.param<std::string>("world_frame", mMapFrameId, "map");
        mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");

        int dictionaryNumber;
        mPnh.param<int>("dictionary", dictionaryNumber, static_cast<int>(cv::aruco::DICT_4X4_50));
        mPnh.param<int>("min_hit_count_before_publish", mMinHitCountBeforePublish, 5);
        mPnh.param<int>("max_hit_count", mMaxHitCount, 5);
        mPnh.param<int>("increment_weight", mTagIncrementWeight, 2);
        mPnh.param<int>("decrement_weight", mTagDecrementWeight, 1);
        mDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dictionaryNumber));

        mDetectedImagePub = mNh.advertise<sensor_msgs::Image>("tag_detection", 1);

        mServiceEnableDetections = mNh.advertiseService("enable_detections", &TagDetectorNodeletBase::enableDetectionsCallback, this);

        // Lambda handles passing class pointer (implicit first parameter) to configCallback
        mCallbackType = [this](DetectorParamsConfig const& config, uint32_t level) { configCallback(config, level); };
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

        NODELET_INFO_STREAM(std::format("Tag detection ready, min hit count: {}, max hit count: {}, hit increment weight: {}, hit decrement weight: {}",
                                        mMinHitCountBeforePublish, mMaxHitCount, mTagIncrementWeight, mTagDecrementWeight));
    }

    auto TagDetectorNodeletBase::configCallback(DetectorParamsConfig const& config, uint32_t level) const -> void {
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

    auto TagDetectorNodeletBase::enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) -> bool {
        mEnableDetections = req.data;
        if (mEnableDetections) {
            res.message = "Enabled tag detections";
            NODELET_INFO("Enabled tag detections");
        } else {
            res.message = "Disabled tag detections";
            NODELET_INFO("Disabled tag detections");
        }

        res.success = true;
        return true;
    }

    auto StereoTagDetectorNodelet::onInit() -> void {
        TagDetectorNodeletBase::onInit();

        mSensorSub = mNh.subscribe("camera/left/points", 1, &StereoTagDetectorNodelet::pointCloudCallback, this);
    }

    auto ImageTagDetectorNodelet::onInit() -> void {
        TagDetectorNodeletBase::onInit();

        mPnh.param<float>("long_range_camera/fov", mCameraHorizontalFov, 80.0);

        mSensorSub = mNh.subscribe("long_range_camera/image", 1, &ImageTagDetectorNodelet::imageCallback, this);

        mTargetsPub = mNh.advertise<ImageTargets>("tags", 1);
    }


} // namespace mrover
