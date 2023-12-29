#pragma once

#include <charconv>
#include <chrono>
#include <execution>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <thread>
#include <unordered_set>

#include <boost_cpp23_workaround.hpp>

#include <boost/container/static_vector.hpp>
#include <boost/process.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <GL/glew.h>

#include <GL/gl.h>

#include <SDL2/SDL.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "imgui/backends/imgui_impl_opengl3.h"
#include "imgui/backends/imgui_impl_sdl2.h"

#include <unsupported/Eigen/OpenGLSupport>

#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <btBulletDynamicsCommon.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

// #include <moteus/moteus.h>

#include <loop_profiler.hpp>
#include <params_utils.hpp>
#include <point.hpp>
#include <se3.hpp>
#include <units.hpp>
// #include <messaging.hpp>

#include <mrover/CAN.h>
#include <mrover/SimulatorParamsConfig.h>