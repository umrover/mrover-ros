cmake_minimum_required(VERSION 3.16)
project(imgui VERSION 1.90 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

file(GLOB_RECURSE IMGUI_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
add_library(imgui ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(imgui PUBLIC glfw webgpu)
