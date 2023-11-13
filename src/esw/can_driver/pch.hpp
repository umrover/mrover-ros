#pragma once

#include <bit>
#include <bitset>
#include <cctype>
#include <cstdint>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/this_node.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/basic_stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

#include <bimap.hpp>
#include <mrover/CAN.h>
#include <params_utils.hpp>
