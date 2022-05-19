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


/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, std::vector<cv::Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.emplace_back(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.emplace_back(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.emplace_back(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.emplace_back(-markerLength / 2.f, -markerLength / 2.f, 0);
}

// Euclidean distance between two points
static double dist(const cv::Point2f& p1, const cv::Point2f& p2) {
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx * dx + dy * dy);
}

static cv::Point2f getTagCentroidFromCorners(fiducial_msgs::Fiducial fiducial) {
    auto centerX = static_cast<float>((fiducial.x0 + fiducial.x1 + fiducial.x2 + fiducial.x3) / 4.0);
    auto centerY = static_cast<float>((fiducial.y0 + fiducial.y1 + fiducial.y2 + fiducial.y3) / 4.0);
    return {centerX, centerY};
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f>& pts) {
    const cv::Point2f& p0 = pts.at(0);
    const cv::Point2f& p1 = pts.at(1);
    const cv::Point2f& p2 = pts.at(2);
    const cv::Point2f& p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
    a2 = sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));
    return a1 + a2;
}

// estimate reprojection error
static double getReprojectionError(const std::vector<cv::Point3f>& objectPoints,
                                   const std::vector<cv::Point2f>& imagePoints,
                                   const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                   const cv::Vec3d& rvec, const cv::Vec3d& tvec) {

    std::vector<cv::Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i = 0; i < objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error * error;
    }
    double reprojection_error = totalError / (double) objectPoints.size();
    return reprojection_error;
}

void FiducialsNode::estimatePoseSingleMarkers(float markerLength,
                                              const cv::Mat& cameraMatrix,
                                              const cv::Mat& distCoeffs,
                                              std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                                              std::vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    std::vector<cv::Point3f> markerObjPoints;
    int nMarkers = (int) corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
        double fiducialSize = markerLength;

        auto fiducialIt = fiducialLens.find(ids[i]);
        if (fiducialIt != fiducialLens.end()) {
            fiducialSize = fiducialIt->second;
        }

        getSingleMarkerObjectPoints(static_cast<float>(fiducialSize), markerObjPoints);
        cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                     rvecs[i], tvecs[i]);

        reprojectionError[i] = getReprojectionError(markerObjPoints, corners[i],
                                                    cameraMatrix, distCoeffs,
                                                    rvecs[i], tvecs[i]);
    }
}

void FiducialsNode::configCallback(mrover::DetectorParamsConfig& config, uint32_t level) {
    /* Don't load initial config, since it will overwrite the rosparam settings */
    if (level == 0xFFFFFFFF) {
        return;
    }

    detectorParams->adaptiveThreshConstant = config.adaptiveThreshConstant;
    detectorParams->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
    detectorParams->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
    detectorParams->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
    detectorParams->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
    detectorParams->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
    detectorParams->cornerRefinementWinSize = config.cornerRefinementWinSize;
#if CV_MINOR_VERSION == 2 and CV_MAJOR_VERSION == 3
    detectorParams->doCornerRefinement = config.doCornerRefinement;
#else
    if (config.doCornerRefinement) {
        if (config.cornerRefinementSubpix) {
            detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        } else {
            detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        }
    } else {
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif
    detectorParams->errorCorrectionRate = config.errorCorrectionRate;
    detectorParams->minCornerDistanceRate = config.minCornerDistanceRate;
    detectorParams->markerBorderBits = config.markerBorderBits;
    detectorParams->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
    detectorParams->minDistanceToBorder = config.minDistanceToBorder;
    detectorParams->minMarkerDistanceRate = config.minMarkerDistanceRate;
    detectorParams->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
    detectorParams->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
    detectorParams->minOtsuStdDev = config.minOtsuStdDev;
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
    detectorParams->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
}

void FiducialsNode::ignoreCallback(const std_msgs::String& msg) {
    ignoreIds.clear();
    pnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

void FiducialsNode::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    if (haveCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.at<double>(i, j) = msg->K[i * 3 + j];
            }
        }

        for (int i = 0; i < 5; i++) {
            distortionCoeffs.at<double>(0, i) = msg->D[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    } else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!enable_detections) {
        return; //return without doing anything
    }

    if (verbose) {
        ROS_INFO("Got image %d", msg->header.seq);
    }

    fiducial_msgs::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id = frameId;
    fva.image_seq = static_cast<int>(msg->header.seq);

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);

        int detected_count = (int) ids.size();
        if (verbose || detected_count != prev_detected_count) {
            prev_detected_count = detected_count;
            ROS_INFO("Detected %d markers", detected_count);
        }

        for (size_t i = 0; i < ids.size(); i++) {
            if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                if (verbose) {
                    ROS_INFO("Ignoring id %d", ids[i]);
                }
                continue;
            }
            fiducial_msgs::Fiducial fid;
            fid.fiducial_id = ids[i];

            fid.x0 = corners[i][0].x;
            fid.y0 = corners[i][0].y;
            fid.x1 = corners[i][1].x;
            fid.y1 = corners[i][1].y;
            fid.x2 = corners[i][2].x;
            fid.y2 = corners[i][2].y;
            fid.x3 = corners[i][3].x;
            fid.y3 = corners[i][3].y;
            fva.fiducials.push_back(fid);
        }

        vertices_pub.publish(fva);

        if (fva.fiducials.empty()) {
            // No tags found, return both tags as invalid
            markerLocations.first.x = markerLocations.first.y = -1;
            markerLocations.second.x = markerLocations.second.y = -1;
        } else if (fva.fiducials.size() == 1) {
            // One tag found, set second as invalid
            markerLocations.first = getTagCentroidFromCorners(fva.fiducials[0]);
            markerLocations.second.x = markerLocations.second.y = -1;
        } else {
            // Detected >= 2 tags
            // If >= 3 tags, return the leftmost and rightmost detected tags
            // to account for potentially seeing 2 of each tag on a post
            std::vector<cv::Point2f> tagLocations;
            tagLocations.reserve(fva.fiducials.size());
            for (auto tag: fva.fiducials) {
                tagLocations.push_back(getTagCentroidFromCorners(tag));
            }
            std::sort(tagLocations.begin(), tagLocations.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.x < rhs.x;
                      });
            markerLocations.first = tagLocations[0];
            markerLocations.second = tagLocations[tagLocations.size() - 1];
        }

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (publish_images) {
            image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception& e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

std::pair<double, double> getDistAndBearingFromPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg, int u, int v) {
    // Could be done using PCL point clouds instead
    size_t arrayPos = v * msg->row_step + u * msg->point_step;
    size_t arrayPosX = arrayPos + msg->fields[0].offset;
    size_t arrayPosY = arrayPos + msg->fields[1].offset;
    size_t arrayPosZ = arrayPos + msg->fields[2].offset;

    float x, y, z;
    memcpy(&x, &msg->data[arrayPosX], sizeof(float));
    memcpy(&y, &msg->data[arrayPosY], sizeof(float));
    memcpy(&z, &msg->data[arrayPosZ], sizeof(float));

    // Ensure xyz are valid
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) return {-1, -1};

    const float PI = 3.14159f;
    double distance = std::sqrt(x * x + y * y + z * z);
    double bearing = std::atan2(x, z) * 180.0 / PI;

    return {distance, bearing};
}

void FiducialsNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    mrover::MarkerInfo detectedMarkerInfo;
    detectedMarkerInfo.distance1 = detectedMarkerInfo.bearing1 = -1;
    detectedMarkerInfo.distance2 = detectedMarkerInfo.bearing2 = -1;

    if (markerLocations.first.x != -1 && markerLocations.first.y != -1) {
        // Get the x/y/z from this point in the pointcloud
        auto x = static_cast<int>(std::lround(markerLocations.first.x));
        auto y = static_cast<int>(std::lround(markerLocations.first.y));
        std::tie(detectedMarkerInfo.distance1, detectedMarkerInfo.bearing1) = getDistAndBearingFromPointCloud(msg, x, y);
    }
    if (markerLocations.second.x != -1 && markerLocations.second.y != -1) {
        // Get the x/y/z from this point in the pointcloud
        auto x = static_cast<int>(std::lround(markerLocations.second.x));
        auto y = static_cast<int>(std::lround(markerLocations.second.y));
        std::tie(detectedMarkerInfo.distance2, detectedMarkerInfo.bearing2) = getDistAndBearingFromPointCloud(msg, x, y);
    }
    markerInfo_pub.publish(detectedMarkerInfo);
}

void FiducialsNode::poseEstimateCallback(const FiducialArrayConstPtr& msg) {
    std::vector<cv::Vec3d> rvecs, tvecs;

    vision_msgs::Detection2DArray vma;
    fiducial_msgs::FiducialTransformArray fta;
    if (vis_msgs) {
        vma.header.stamp = msg->header.stamp;
        vma.header.frame_id = frameId;
        vma.header.seq = msg->header.seq;
    } else {
        fta.header.stamp = msg->header.stamp;
        fta.header.frame_id = frameId;
        fta.image_seq = static_cast<int>(msg->header.seq);
    }
    frameNum++;

    if (doPoseEstimation) {
        try {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    ROS_ERROR("No camera intrinsics");
                }
                return;
            }

            std::vector<double> reprojectionError;
            estimatePoseSingleMarkers((float) fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i = 0; i < ids.size(); i++) {
                cv::aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float) fiducial_len);
                if (verbose) {
                    ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                             tvecs[i][0], tvecs[i][1], tvecs[i][2],
                             rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                }

                if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                    if (verbose) {
                        ROS_INFO("Ignoring id %d", ids[i]);
                    }
                    continue;
                }

                double angle = norm(rvecs[i]);
                cv::Vec3d axis = rvecs[i] / angle;

                if (verbose) {
                    ROS_INFO("angle %f axis %f %f %f",
                             angle, axis[0], axis[1], axis[2]);
                }

                double object_error =
                        (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                        (norm(tvecs[i]) / fiducial_len);

                // Standard ROS vision_msgs
                fiducial_msgs::FiducialTransform ft;
                tf2::Quaternion q;
                if (vis_msgs) {
                    vision_msgs::Detection2D vm;
                    vision_msgs::ObjectHypothesisWithPose vmh;
                    vmh.id = ids[i];
                    vmh.score = exp(-2 * object_error); // [0, infinity] -> [1,0]
                    vmh.pose.pose.position.x = tvecs[i][0];
                    vmh.pose.pose.position.y = tvecs[i][1];
                    vmh.pose.pose.position.z = tvecs[i][2];
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    vmh.pose.pose.orientation.w = q.w();
                    vmh.pose.pose.orientation.x = q.x();
                    vmh.pose.pose.orientation.y = q.y();
                    vmh.pose.pose.orientation.z = q.z();

                    vm.results.push_back(vmh);
                    vma.detections.push_back(vm);
                } else {
                    ft.fiducial_id = ids[i];

                    ft.transform.translation.x = tvecs[i][0];
                    ft.transform.translation.y = tvecs[i][1];
                    ft.transform.translation.z = tvecs[i][2];
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    ft.transform.rotation.w = q.w();
                    ft.transform.rotation.x = q.x();
                    ft.transform.rotation.y = q.y();
                    ft.transform.rotation.z = q.z();
                    ft.fiducial_area = calcFiducialArea(corners[i]);
                    ft.image_error = reprojectionError[i];
                    // Convert image_error (in pixels) to object_error (in meters)
                    ft.object_error =
                            (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                            (norm(tvecs[i]) / fiducial_len);

                    fta.transforms.push_back(ft);
                }

                // Publish tf for the fiducial relative to the camera
                if (publishFiducialTf) {
                    if (vis_msgs) {
                        geometry_msgs::TransformStamped ts;
                        ts.transform.translation.x = tvecs[i][0];
                        ts.transform.translation.y = tvecs[i][1];
                        ts.transform.translation.z = tvecs[i][2];
                        ts.transform.rotation.w = q.w();
                        ts.transform.rotation.x = q.x();
                        ts.transform.rotation.y = q.y();
                        ts.transform.rotation.z = q.z();
                        ts.header.frame_id = frameId;
                        ts.header.stamp = msg->header.stamp;
                        ts.child_frame_id = "fiducial_" + std::to_string(ids[i]);
                        broadcaster.sendTransform(ts);
                    } else {
                        geometry_msgs::TransformStamped ts;
                        ts.transform = ft.transform;
                        ts.header.frame_id = frameId;
                        ts.header.stamp = msg->header.stamp;
                        ts.child_frame_id = "fiducial_" + std::to_string(ft.fiducial_id);
                        broadcaster.sendTransform(ts);
                    }
                }
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (cv::Exception& e) {
            ROS_ERROR("cv exception: %s", e.what());
        }
    }
    if (vis_msgs)
        pose_pub.publish(vma);
    else
        pose_pub.publish(fta);
}

void FiducialsNode::handleIgnoreString(const std::string& str) {
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
                ignoreIds.push_back(j);
            }
        } else if (range.size() == 1) {
            int fid = std::stoi(range[0]);
            ROS_INFO("Ignoring fiducial id %d", fid);
            ignoreIds.push_back(fid);
        } else {
            ROS_ERROR("Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool FiducialsNode::enableDetectionsCallback(std_srvs::SetBool::Request& req,
                                             std_srvs::SetBool::Response& res) {
    enable_detections = req.data;
    if (enable_detections) {
        res.message = "Enabled aruco detections.";
        ROS_INFO("Enabled aruco detections.");
    } else {
        res.message = "Disabled aruco detections.";
        ROS_INFO("Disabled aruco detections.");
    }

    res.success = true;
    return true;
}


FiducialsNode::FiducialsNode() : nh(), pnh("~"), it(nh) {
    frameNum = 0;
    prev_detected_count = -1;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicNo;

    detectorParams = new cv::aruco::DetectorParameters();

    pnh.param<bool>("publish_images", publish_images, true);
    pnh.param<double>("fiducial_len", fiducial_len, 0.14);
    pnh.param<int>("dictionary", dicNo, 7);
    pnh.param<bool>("do_pose_estimation", doPoseEstimation, false);
    pnh.param<bool>("publish_fiducial_tf", publishFiducialTf, true);
    pnh.param<bool>("vis_msgs", vis_msgs, false);
    pnh.param<bool>("verbose", verbose, false);

    std::string str;
    std::vector<std::string> strs;

    pnh.param<std::string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    // fiducial size can take comma separated list of size: id or size: range,
    // e.g. "200.0: 12, 300.0: 200-300"
    pnh.param<std::string>("fiducial_len_override", str, "");
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
                    fiducialLens[j] = len;
                }
            } else if (range.size() == 1) {
                int fid = std::stoi(range[0]);
                ROS_INFO("Setting fiducial id %d length to %f", fid, len);
                fiducialLens[fid] = len;
            } else {
                ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
            }
        } else {
            ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    image_pub = it.advertise("/fiducial_images", 1);

    markerInfo_pub = nh.advertise<mrover::MarkerInfo>("marker_info", 1);

    vertices_pub = nh.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 1);

    if (vis_msgs)
        pose_pub = nh.advertise<vision_msgs::Detection2DArray>("fiducial_transforms", 1);
    else
        pose_pub = nh.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 1);

    dictionary = cv::aruco::getPredefinedDictionary(dicNo);

    img_sub = it.subscribe("/camera/color/image_raw", 1, &FiducialsNode::imageCallback, this);

    pc_sub = nh.subscribe("/camera/depth/points", 1, &FiducialsNode::pointCloudCallback, this);

    vertices_sub = nh.subscribe("/fiducial_vertices", 1, &FiducialsNode::poseEstimateCallback, this);
    caminfo_sub = nh.subscribe("/camera/color/camera_info", 1, &FiducialsNode::camInfoCallback, this);

    ignore_sub = nh.subscribe("/ignore_fiducials", 1, &FiducialsNode::ignoreCallback, this);

    service_enable_detections = nh.advertiseService("enable_detections", &FiducialsNode::enableDetectionsCallback, this);

    callbackType = [this](mrover::DetectorParamsConfig& config, uint32_t level) { configCallback(config, level); };
    configServer.setCallback(callbackType);

    pnh.param<double>("adaptiveThreshConstant", detectorParams->adaptiveThreshConstant, 7);
    pnh.param<int>("adaptiveThreshWinSizeMax", detectorParams->adaptiveThreshWinSizeMax, 53); /* default 23 */
    pnh.param<int>("adaptiveThreshWinSizeMin", detectorParams->adaptiveThreshWinSizeMin, 3);
    pnh.param<int>("adaptiveThreshWinSizeStep", detectorParams->adaptiveThreshWinSizeStep, 4); /* default 10 */
    pnh.param<int>("cornerRefinementMaxIterations", detectorParams->cornerRefinementMaxIterations, 30);
    pnh.param<double>("cornerRefinementMinAccuracy", detectorParams->cornerRefinementMinAccuracy, 0.01); /* default 0.1 */
    pnh.param<int>("cornerRefinementWinSize", detectorParams->cornerRefinementWinSize, 5);
    bool doCornerRefinement = true;
    pnh.param<bool>("doCornerRefinement", doCornerRefinement, true);
    if (doCornerRefinement) {
        bool cornerRefinementSubPix = true;
        pnh.param<bool>("cornerRefinementSubPix", cornerRefinementSubPix, true);
        if (cornerRefinementSubPix) {
            detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        } else {
            detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        }
    } else {
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
    pnh.param<double>("errorCorrectionRate", detectorParams->errorCorrectionRate, 0.6);
    pnh.param<double>("minCornerDistanceRate", detectorParams->minCornerDistanceRate, 0.05);
    pnh.param<int>("markerBorderBits", detectorParams->markerBorderBits, 1);
    pnh.param<double>("maxErroneousBitsInBorderRate", detectorParams->maxErroneousBitsInBorderRate, 0.04);
    pnh.param<int>("minDistanceToBorder", detectorParams->minDistanceToBorder, 3);
    pnh.param<double>("minMarkerDistanceRate", detectorParams->minMarkerDistanceRate, 0.05);
    pnh.param<double>("minMarkerPerimeterRate", detectorParams->minMarkerPerimeterRate, 0.1); /* default 0.3 */
    pnh.param<double>("maxMarkerPerimeterRate", detectorParams->maxMarkerPerimeterRate, 4.0);
    pnh.param<double>("minOtsuStdDev", detectorParams->minOtsuStdDev, 5.0);
    pnh.param<double>("perspectiveRemoveIgnoredMarginPerCell", detectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    pnh.param<int>("perspectiveRemovePixelPerCell", detectorParams->perspectiveRemovePixelPerCell, 8);
    pnh.param<double>("polygonalApproxAccuracyRate", detectorParams->polygonalApproxAccuracyRate, 0.01); /* default 0.05 */

    ROS_INFO("Aruco detection ready");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detect");

    new boost::shared_ptr<FiducialsNode>{new FiducialsNode()};

    ros::spin();

    return EXIT_SUCCESS;
}
