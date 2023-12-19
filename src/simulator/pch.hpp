#pragma once

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

#include <boost/container/static_vector.hpp>
#include <boost/process.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/serialization.h>
#include <rospack/rospack.h>
#include <urdf/model.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <GL/glew.h>

#include <GL/gl.h>

#include <SDL2/SDL.h>

#include "imgui/backends/imgui_impl_opengl3.h"
#include "imgui/backends/imgui_impl_sdl2.h"

#include <unsupported/Eigen/OpenGLSupport>

#include <btBulletDynamicsCommon.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <params_utils.hpp>
#include <se3.hpp>

#include <mrover/CAN.h>
#include <mrover/SimulatorParamsConfig.h>
