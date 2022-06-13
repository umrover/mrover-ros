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
#include "aruco_detect.hpp"

void FiducialsNode::configCallback(mrover::DetectorParamsConfig& config, uint32_t level) {
    /* Don't load initial config, since it will overwrite the rosparam settings */
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

void FiducialsNode::ignoreCallback(std_msgs::String const& msg) {
    mIgnoreIds.clear();
    mPnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

void FiducialsNode::camInfoCallback(sensor_msgs::CameraInfo::ConstPtr const& msg) {
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

vision_msgs::ObjectHypothesisWithPose getObjectHypothesis(Fiducial const& fid) {
    vision_msgs::ObjectHypothesisWithPose vmh;
    vmh.id = fid.id;
    vmh.score = 0.0;
    vmh.pose.pose.position.x = fid.worldPosition->x;
    vmh.pose.pose.position.y = fid.worldPosition->y;
    vmh.pose.pose.position.z = fid.worldPosition->z;
    vmh.pose.pose.orientation.w = 1.0;
    vmh.pose.pose.orientation.x = 0.0;
    vmh.pose.pose.orientation.y = 0.0;
    vmh.pose.pose.orientation.z = 0.0;
    return vmh;
}

void FiducialsNode::poseEstimateCallback(FiducialArrayConstPtr const& msg) {
    std::vector<cv::Vec3d> rvecs, tvecs;

    vision_msgs::Detection2DArray vma;
    vma.header.stamp = msg->header.stamp;
    vma.header.frame_id = mFrameId;
    vma.header.seq = msg->header.seq;
    mFrameNum++;

    if (mDoPoseEstimation) {
        try {
            if (!mHasCamInfo) {
                if (mFrameNum > 5) ROS_ERROR("No camera intrinsics");
                return;
            }

            std::vector<Fiducial> readyFiducials;
            for (auto& [id, fd]: mFiducials) {
                if (fd.imageCenter && fd.worldPosition) {
                    readyFiducials.push_back(fd);
                }
            }

            auto [leftFidIt, rightFidIt] = std::minmax_element(
                    readyFiducials.begin(), readyFiducials.end(),
                    [](const Fiducial& fid1, const Fiducial& fid2) {
                        return fid1.imageCenter->x < fid2.imageCenter->x;
                    });

            for (auto it: {leftFidIt, rightFidIt}) {
                if (it == readyFiducials.end()) continue;

                Fiducial const& fid = *it;

                if (std::count(mIgnoreIds.begin(), mIgnoreIds.end(), fid.id)) {
                    if (mIsVerbose) ROS_INFO("Ignoring id %d", fid.id);
                    continue;
                }

                // Standard ROS vision_msgs
                vision_msgs::Detection2D vm;
                vm.results.push_back(getObjectHypothesis(fid));
                vma.detections.push_back(vm);
            }
        } catch (cv_bridge::Exception const& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (cv::Exception const& e) {
            ROS_ERROR("cv exception: %s", e.what());
        }
    }
    mPosePub.publish(vma);
}

void FiducialsNode::handleIgnoreString(std::string const& str) {
    // ignore fiducials can take comma separated list of individual
    // fiducial ids or ranges, eg "1,4,8,9-12,30-40"
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

bool FiducialsNode::enableDetectionsCallback(std_srvs::SetBool::Request& req,
                                             std_srvs::SetBool::Response& res) {
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


FiducialsNode::FiducialsNode() : mNh(), mPnh("~"), mIt(mNh) {
    mFrameNum = 0;
    mPrevDetectedCount = -1;

    // Camera intrinsics
    mCamMat = cv::Mat::zeros(3, 3, CV_64F);
    // distortion coefficients
    mDistCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    mHasCamInfo = false;
    mEnableDetections = true;

    int dicNo;

    mDetectorParams = new cv::aruco::DetectorParameters();

    mPnh.param<bool>("publish_images", mPublishImages, true);
    mPnh.param<double>("fiducial_len", mFiducialLen, 0.2);
    mPnh.param<int>("dictionary", dicNo, 0);
    mPnh.param<bool>("do_pose_estimation", mDoPoseEstimation, true);
    mPnh.param<bool>("publish_fiducial_tf", mPublishFiducialTf, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    std::string str;
    std::vector<std::string> strs;

    mPnh.param<std::string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    // fiducial size can take comma separated list of size: id or size: range,
    // e.g. "200.0: 12, 300.0: 200-300"
    mPnh.param<std::string>("fiducial_len_override", str, "");
    boost::split(strs, str, boost::is_any_of(","));
    for (const std::string& element: strs) {
        if (element.empty()) {
            continue;
        }
        std::vector<std::string> parts;
        boost::split(parts, element, boost::is_any_of(":"));
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            boost::split(range, element, boost::is_any_of("-"));
            if (range.size() == 2) {
                int start = std::stoi(range[0]);
                int end = std::stoi(range[1]);
                ROS_INFO("Setting fiducial id range %d - %d length to %f",
                         start, end, len);
                for (int j = start; j <= end; j++) {
                    mFiducialLens[j] = len;
                }
            } else if (range.size() == 1) {
                int fid = std::stoi(range[0]);
                ROS_INFO("Setting fiducial id %d length to %f", fid, len);
                mFiducialLens[fid] = len;
            } else {
                ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
            }
        } else {
            ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    mImgPub = mIt.advertise("fiducial_images", 1);
    mVerticesPub = mNh.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 1);
    mPosePub = mNh.advertise<vision_msgs::Detection2DArray>("fiducial_transforms", 1);
    mDictionary = cv::aruco::getPredefinedDictionary(dicNo);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedValue"
    mImgSub = mIt.subscribe("camera/color/image_raw", 1, &FiducialsNode::imageCallback, this);
    mPcSub = mNh.subscribe("camera/depth/points", 1, &FiducialsNode::pointCloudCallback, this);
    mVerticesSub = mNh.subscribe("fiducial_vertices", 1, &FiducialsNode::poseEstimateCallback, this);
    mCamInfoSub = mNh.subscribe("camera/color/camera_info", 1, &FiducialsNode::camInfoCallback, this);
    mIgnoreSub = mNh.subscribe("ignore_fiducials", 1, &FiducialsNode::ignoreCallback, this);
    mServiceEnableDetections = mNh.advertiseService("enable_detections", &FiducialsNode::enableDetectionsCallback, this);
#pragma clang diagnostic pop

    // Lambda handles passing class pointer (implicit first parameter) to configCallback
    mCallbackType = [this](mrover::DetectorParamsConfig& config, uint32_t level) { configCallback(config, level); };
    mConfigServer.setCallback(mCallbackType);

    mPnh.param<double>("adaptiveThreshConstant", mDetectorParams->adaptiveThreshConstant, 7);
    mPnh.param<int>("adaptiveThreshWinSizeMax", mDetectorParams->adaptiveThreshWinSizeMax, 53); /* default 23 */
    mPnh.param<int>("adaptiveThreshWinSizeMin", mDetectorParams->adaptiveThreshWinSizeMin, 3);
    mPnh.param<int>("adaptiveThreshWinSizeStep", mDetectorParams->adaptiveThreshWinSizeStep, 4); /* default 10 */
    mPnh.param<int>("cornerRefinementMaxIterations", mDetectorParams->cornerRefinementMaxIterations, 30);
    mPnh.param<double>("cornerRefinementMinAccuracy", mDetectorParams->cornerRefinementMinAccuracy, 0.01); /* default 0.1 */
    mPnh.param<int>("cornerRefinementWinSize", mDetectorParams->cornerRefinementWinSize, 5);

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
    mPnh.param<double>("errorCorrectionRate", mDetectorParams->errorCorrectionRate, 0.6);
    mPnh.param<double>("minCornerDistanceRate", mDetectorParams->minCornerDistanceRate, 0.05);
    mPnh.param<int>("markerBorderBits", mDetectorParams->markerBorderBits, 1);
    mPnh.param<double>("maxErroneousBitsInBorderRate", mDetectorParams->maxErroneousBitsInBorderRate, 0.04);
    mPnh.param<int>("minDistanceToBorder", mDetectorParams->minDistanceToBorder, 3);
    mPnh.param<double>("minMarkerDistanceRate", mDetectorParams->minMarkerDistanceRate, 0.05);
    mPnh.param<double>("minMarkerPerimeterRate", mDetectorParams->minMarkerPerimeterRate, 0.1); /* default 0.3 */
    mPnh.param<double>("maxMarkerPerimeterRate", mDetectorParams->maxMarkerPerimeterRate, 4.0);
    mPnh.param<double>("minOtsuStdDev", mDetectorParams->minOtsuStdDev, 5.0);
    mPnh.param<double>("perspectiveRemoveIgnoredMarginPerCell", mDetectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    mPnh.param<int>("perspectiveRemovePixelPerCell", mDetectorParams->perspectiveRemovePixelPerCell, 8);
    mPnh.param<double>("polygonalApproxAccuracyRate", mDetectorParams->polygonalApproxAccuracyRate, 0.01); /* default 0.05 */

    ROS_INFO("Aruco detection ready");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detect");

    new boost::shared_ptr<FiducialsNode>{new FiducialsNode()};

    ros::spin();

    return EXIT_SUCCESS;
}
