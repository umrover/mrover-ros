#pragma once

#include <filesystem>
#include <format>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <fstream>
#include <source_location>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/serialization.h>
#include <urdf/model.h>

#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL2/SDL.h>

#include <se3.hpp>
#include <unsupported/Eigen/OpenGLSupport>

#include <params_utils.hpp>

#include <mrover/SimulatorParamsConfig.h>

#include <boost/container/static_vector.hpp>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
