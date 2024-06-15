// ROS INCLUDES
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <ros/subscriber.h>


// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// MRover
#include "point.hpp"

// STL
#include <algorithm>
#include <execution>
#include <vector>
