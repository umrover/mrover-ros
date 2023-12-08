#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

#include <stdexcept>
#include <format>
#include <iostream>
#include <memory>
#include <thread>
#include <filesystem>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>

#include <ros/serialization.h>
#include <urdf/model.h>

#include <params_utils.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
