/*
 * Copyright (c) 2017-20, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include "tag_detector.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void FiducialsNode::configCallback(mrover::DetectorParamsConfig &config, uint32_t level) {
    // Don't load initial config, since it will overwrite the rosparam settings
    if (level == 0xFFFFFFFF) {
        return;
    }

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

void FiducialsNode::ignoreCallback(std_msgs::String const &msg) {
    mIgnoreIds.clear();
    mPnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

void FiducialsNode::camInfoCallback(sensor_msgs::CameraInfo::ConstPtr const &msg) {
    if (mHasCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mCamMat.at<double>(i, j) = msg->K[i * 3 + j];
            }
        }

        for (int i = 0; i < 5; i++) {
            mDistCoeffs.at<double>(0, i) = msg->D[i];
        }

        mHasCamInfo = true;
        mFrameId = msg->header.frame_id;
    } else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void FiducialsNode::handleIgnoreString(std::string const &str) {
    // Ignore fiducials can take comma separated list of individual
    // Tag ids or ranges, eg "1,4,8,9-12,30-40"
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    for (const std::string &element: strs) {
        if (element.empty()) {
            continue;
        }
        std::vector<std::string> range;
        boost::split(range, element, boost::is_any_of("-"));
        if (range.size() == 2) {
            int start = std::stoi(range[0]);
            int end = std::stoi(range[1]);
            ROS_INFO("Ignoring fiducial id range %d to %d", start, end);
            for (int j = start; j <= end; j++) {
                mIgnoreIds.push_back(j);
            }
        } else if (range.size() == 1) {
            int fid = std::stoi(range[0]);
            ROS_INFO("Ignoring fiducial id %d", fid);
            mIgnoreIds.push_back(fid);
        } else {
            ROS_ERROR("Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool FiducialsNode::enableDetectionsCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    mEnableDetections = req.data;
    if (mEnableDetections) {
        res.message = "Enabled aruco detections.";
        ROS_INFO("Enabled aruco detections.");
    } else {
        res.message = "Disabled aruco detections.";
        ROS_INFO("Disabled aruco detections.");
    }

    res.success = true;
    return true;
}

FiducialsNode::FiducialsNode() : mNh(), mPnh("~"), mIt(mNh), mTfListener(mTfBuffer) {
    // Camera intrinsics
    mCamMat = cv::Mat::zeros(3, 3, CV_64F);
    // distortion coefficients
    mDistCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    mDetectorParams = new cv::aruco::DetectorParameters();
    auto defaultDetectorParams = cv::aruco::DetectorParameters::create();

    int dicNo;
    mPnh.param<bool>("publish_images", mPublishImages, true);
    mPnh.param<int>("dictionary", dicNo, 0);
    mPnh.param<bool>("publish_fiducial_tf", mPublishFiducialTf, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    std::string str;

    mPnh.param<std::string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    mImgPub = mIt.advertise("fiducial_images", 1);
    mDictionary = cv::aruco::getPredefinedDictionary(dicNo);

    mImgSub = mIt.subscribe("camera/color/image_raw", 1, &FiducialsNode::imageCallback, this);
    mPcSub = mNh.subscribe("camera/depth/points", 1, &FiducialsNode::pointCloudCallback, this);
    mCamInfoSub = mNh.subscribe("camera/color/camera_info", 1, &FiducialsNode::camInfoCallback, this);
    mIgnoreSub = mNh.subscribe("ignore_fiducials", 1, &FiducialsNode::ignoreCallback, this);
    mServiceEnableDetections = mNh.advertiseService("enable_detections", &FiducialsNode::enableDetectionsCallback,
                                                    this);

    // Lambda handles passing class pointer (implicit first parameter) to configCallback
    mCallbackType = [this](mrover::DetectorParamsConfig &config, uint32_t level) { configCallback(config, level); };
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

    ROS_INFO("Aruco detection ready");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_detect");

    [[maybe_unused]] auto node = std::make_unique<FiducialsNode>();

    ros::spin();

    return EXIT_SUCCESS;
}

void XYZFilter::addReading(SE3 const &fidInOdom) {
    fidInOdomX.push(fidInOdom.positionVector().x());
    fidInOdomY.push(fidInOdom.positionVector().y());
    fidInOdomZ.push(fidInOdom.positionVector().z());
}

void XYZFilter::setFilterParams(size_t count, double proportion) {
    fidInOdomX.setFilterCount(count);
    fidInOdomX.setProportion(static_cast<float>(proportion));
    fidInOdomY.setFilterCount(count);
    fidInOdomY.setProportion(static_cast<float>(proportion));
    fidInOdomZ.setFilterCount(count);
    fidInOdomZ.setProportion(static_cast<float>(proportion));
}

bool XYZFilter::ready() const {
    return fidInOdomX.ready();
}

SE3 XYZFilter::getFidInOdom() const {
    return {{fidInOdomX.get(), fidInOdomY.get(), fidInOdomZ.get()}, Eigen::Quaterniond::Identity()};
}
