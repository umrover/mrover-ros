// STL
#include <iostream>
#include <string>
#include <execution>
#include <format>

// ROS
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <ros/subscriber.h>
#include <tf/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// GStreamer
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
