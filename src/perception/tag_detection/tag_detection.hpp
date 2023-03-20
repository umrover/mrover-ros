#pragma once

#include <se3.hpp>

#include <pcl_ros/point_cloud.h>

#include <opencv2/core/types.hpp>

#include <memory>
#include <unordered_map>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

struct Tag {
    cv::Point2i center;
    SE3 poseInCamera;
};

using Detections = std::unordered_map<uint8_t, Tag>;

void detectTags(PointCloudPtr const& pointCloud, Detections& detections);
