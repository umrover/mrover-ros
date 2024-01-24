#pragma once

#include <algorithm>
#include <cstdint>

#include <nav_msgs/OccupancyGrid.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <se3.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mrover/DetectorParamsConfig.h>