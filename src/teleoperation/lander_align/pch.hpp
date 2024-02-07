#pragma once

#include <boost_cpp23_workaround.hpp>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//TF
#include <tf/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "lie/se3.hpp"

#include <Eigen/Dense>

#include "point.hpp"