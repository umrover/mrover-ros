#pragma once

#include <thread>
#include <atomic>

#include <nodelet/loader.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include <ros/this_node.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <streaming.hpp>
