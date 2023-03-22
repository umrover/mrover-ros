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

void TagDetectorNode::ignoreCallback(std_msgs::String const& msg) {
    mIgnoreIds.clear();
    mPnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

void TagDetectorNode::handleIgnoreString(std::string const& str) {
    // Ignore fiducials can take comma separated list of individual
    // Tag ids or ranges, eg "1,4,8,9-12,30-40"
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    for (const std::string& element: strs) {
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

    mNh.param<bool>("gps_linearization/use_odom_frame", mUseOdom, false);
    mNh.param<std::string>("odom_frame", mOdomFrameId, "odom");
    mNh.param<std::string>("map_frame", mMapFrameId, "map");
    mNh.param<std::string>("rover_frame", mBaseLinkFrameId, "base_link");

    mPnh.param<bool>("publish_images", mPublishImages, true);
    mPnh.param<int>("dictionary", dictionaryNumber, 0);
    mPnh.param<bool>("publish_fiducial_tf", mPublishFiducialTf, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    std::string str;

    mPnh.param<std::string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    mImgPub = mIt.advertise("tag_detection", 1);
    mDictionary = cv::aruco::getPredefinedDictionary(dictionaryNumber);

    mImgSub = mIt.subscribe("camera/left/image", 1, &TagDetectorNode::imageCallback, this);
    mPcSub = mNh.subscribe("camera/left/points", 1, &TagDetectorNode::pointCloudCallback, this);
    mIgnoreSub = mNh.subscribe("ignore_fiducials", 1, &TagDetectorNode::ignoreCallback, this);
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

TagDetectorNode::TagDetectorNode() : TagDetectorNode(ros::NodeHandle{}, ros::NodeHandle{"~"}) {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_detector");

    [[maybe_unused]] auto node = std::make_unique<TagDetectorNode>();

    ros::spin();

    return EXIT_SUCCESS;
}
