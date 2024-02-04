cmake_minimum_required(VERSION 3.16)
project(mrover VERSION 2024.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json for clangd
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_link_options(-fuse-ld=lld-16) # LLVM lld is faster than GNU ld
endif ()
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(MROVER_CPP_COMPILE_OPTIONS -Wall -Wextra -Werror -pedantic)
endif ()

# ROS packages list
set(MROVER_ROS_DEPENDENCIES
        rospy
        roscpp
        rostest
        nodelet
        std_msgs
        sensor_msgs
        message_generation
        dynamic_reconfigure
        tf2
        tf2_ros
        tf2_geometry_msgs
        gazebo_ros
)

# Search a path for all files matching a glob pattern and extract the filenames
macro(extract_filenames directory_path out_file_names)
    file(GLOB_RECURSE full_paths ${directory_path})
    set(${out_file_names} "")
    foreach (FULL_PATH ${full_paths})
        get_filename_component(FILE_NAME ${FULL_PATH} NAME)
        list(APPEND ${out_file_names} ${FILE_NAME})
    endforeach ()
endmacro()

extract_filenames(msg/*.msg MROVER_MESSAGE_FILES)

extract_filenames(srv/*.srv MROVER_SERVICE_FILES)

set(MROVER_MESSAGE_DEPENDENCIES
        std_msgs
        sensor_msgs
)

set(MROVER_PARAMETERS
        config/DetectorParams.cfg
)

set(MROVER_CMAKE_INCLUDES
        starter_project/autonomy/AutonomyStarterProject.cmake
)

### ====== ###
### Macros ###
### ====== ###

macro(target_rosify target)
    target_link_libraries(${target} PRIVATE ${catkin_LIBRARIES})
    target_include_directories(${target} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} src/util)
    add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${MROVER_CPP_COMPILE_OPTIONS}>)

    # Installing is necessary for roslaunch to find the node
    install(TARGETS ${target}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endmacro()

macro(mrover_add_library name sources includes)
    file(GLOB_RECURSE CPP_LIB_SOURCES ${sources})
    add_library(${name} ${CPP_LIB_SOURCES})
    target_include_directories(${name} PUBLIC ${includes})
    target_rosify(${name})
endmacro()

macro(mrover_add_header_only_library name includes)
    add_library(${name} INTERFACE)
    target_include_directories(${name} INTERFACE ${includes})
endmacro()

macro(mrover_add_node name sources)
    file(GLOB_RECURSE NODE_SOURCES ${sources})
    add_executable(${name} ${NODE_SOURCES})
    target_rosify(${name})
endmacro()

macro(mrover_add_nodelet name sources includes)
    # A nodelet runs inside another process so it is a library
    mrover_add_library(${name}_nodelet ${sources} ${includes})
    # Also add a node for quick debugging
    mrover_add_node(${name}_node ${sources})
    # Explicitly tell CMake to re-build the nodelet when the node is built
    # CMake cannot tell these are dependent since a node dynamically (at runtime) loads the nodelet as a shared library
    add_dependencies(${name}_node ${name}_nodelet)
    # Allows the source code to split based on whether it is a node or a nodelet
    target_compile_definitions(${name}_nodelet PRIVATE MROVER_IS_NODELET)
    # Optional pre-compiled header (PCH) support
    if (ARGV3)
        target_precompile_headers(${name}_node PRIVATE ${ARGV3})
        target_precompile_headers(${name}_nodelet PRIVATE ${ARGV3})
    endif ()
endmacro()

macro(mrover_nodelet_link_libraries name)
    target_link_libraries(${name}_node PRIVATE ${ARGN})
    target_link_libraries(${name}_nodelet PRIVATE ${ARGN})
endmacro()

macro(mrover_nodelet_include_directories name)
    target_include_directories(${name}_node SYSTEM PRIVATE ${ARGN})
    target_include_directories(${name}_nodelet SYSTEM PRIVATE ${ARGN})
endmacro()

macro(mrover_nodelet_defines name)
    target_compile_definitions(${name}_node PRIVATE ${ARGN})
    target_compile_definitions(${name}_nodelet PRIVATE ${ARGN})
endmacro()

macro(mrover_add_gazebo_plugin name sources includes)
    mrover_add_library(${name} ${sources} ${includes})

    # TODO: find a proper variable name that points to /opt/ros/noetic/lib
    target_link_directories(${name} PRIVATE ${GAZEBO_LIBRARY_DIRS} /opt/ros/noetic/lib)
    target_link_libraries(${name} PRIVATE ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
    target_include_directories(${name} SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
    set_target_properties(${name} PROPERTIES CXX_STANDARD 17)
endmacro()

### ============ ###
### Dependencies ###
### ============ ###

find_package(OpenCV REQUIRED)
find_package(ZED 2 QUIET)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(NetLink REQUIRED libnl-3.0)
pkg_search_module(NetLinkRoute REQUIRED libnl-route-3.0)
if (ZED_FOUND)
    # Anything newer than C++17 combined with libstdc++13 is not supported just yet by NVCC (the CUDA compiler)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
    set(CMAKE_CUDA_FLAGS "--diag-suppress 108,68")
    # Jetson Xavier NX is Volta 7.2 architecture
    # Perception Laptop (A4000, Quadro version of RTX 3080) is Ampere 8.6 architecture
    set(CMAKE_CUDA_ARCHITECTURES 72 86)
    enable_language(CUDA)
endif ()

find_package(
        catkin REQUIRED COMPONENTS
        ${MROVER_ROS_DEPENDENCIES}
)

catkin_python_setup()

foreach (MROVER_CMAKE_INCLUDE ${MROVER_CMAKE_INCLUDES})
    include(${MROVER_CMAKE_INCLUDE})
endforeach ()

### ======== ###
### Messages ###
### ======== ###

add_message_files(
        FILES
        ${MROVER_MESSAGE_FILES}
)

add_service_files(
        FILES
        ${MROVER_SERVICE_FILES}
)

generate_messages(
        DEPENDENCIES
        ${MROVER_MESSAGE_DEPENDENCIES}
)

generate_dynamic_reconfigure_options(
        ${MROVER_PARAMETERS}
)

catkin_package()

### ======= ###
### Targets ###
### ======= ###

# Please browse the "Macros" section before adding anything here
# Lots of custom macros have been added to make adding new targets easier

## Libraries

mrover_add_library(lie src/util/lie/*.cpp src/util/lie)

## ESW

mrover_add_header_only_library(moteus deps/moteus/lib/cpp/mjbots)
mrover_add_header_only_library(moteus_fw deps/moteus)
mrover_add_header_only_library(can_device src/esw/can_device)
mrover_add_library(motor_library src/esw/motor_library/*.cpp src/esw/motor_library)
target_link_libraries(motor_library PUBLIC can_device moteus moteus_fw)

# TODO(quintin) Fix this CMake hard coding
mrover_add_nodelet(can_driver src/esw/can_driver/*.cpp src/esw/can_driver src/esw/can_driver/pch.hpp)
mrover_nodelet_link_libraries(can_driver nl-3 nl-route-3)
#mrover_nodelet_link_libraries(can ${NetLink_LIBRARIES} ${NetLinkRoute_LIBRARIES}})
mrover_nodelet_include_directories(can_driver /usr/include/libnl3)

macro(mrover_add_esw_bridge_node name sources)
    mrover_add_node(${name} ${sources})
    target_link_libraries(${name} PRIVATE can_device motor_library)
endmacro()

mrover_add_esw_bridge_node(arm_hw_bridge src/esw/arm_hw_bridge/*.cpp)
mrover_add_esw_bridge_node(arm_translator_bridge src/esw/arm_translator_bridge/*.cpp src/esw/arm_translator_bridge/*.hpp)
mrover_add_esw_bridge_node(cache_bridge src/esw/cache_bridge/*.cpp)
mrover_add_esw_bridge_node(drive_bridge src/esw/drive_bridge/*.cpp)
mrover_add_esw_bridge_node(led_hw_bridge src/esw/led_hw_bridge/*.cpp)
mrover_add_esw_bridge_node(mast_gimbal_bridge src/esw/mast_gimbal_bridge/*.cpp)
mrover_add_esw_bridge_node(pdb_bridge src/esw/pdb_bridge/*.cpp)
mrover_add_esw_bridge_node(sa_bridge src/esw/sa_bridge/*.cpp)
mrover_add_esw_bridge_node(science_bridge src/esw/science_bridge/*.cpp)
mrover_add_esw_bridge_node(brushless_test_bridge src/esw/brushless_test_bridge/*.cpp motor_library)
mrover_add_esw_bridge_node(brushed_test_bridge src/esw/brushed_test_bridge/*.cpp)
mrover_add_esw_bridge_node(test_arm_bridge src/esw/test_arm_bridge/*.cpp)
mrover_add_esw_bridge_node(pdb_test_bridge src/esw/pdb_test_bridge/*.cpp)

## Perception

mrover_add_nodelet(tag_detector src/perception/tag_detector/*.cpp src/perception/tag_detector src/perception/tag_detector/pch.hpp)
mrover_nodelet_link_libraries(tag_detector opencv_core opencv_objdetect opencv_aruco opencv_imgproc tbb lie)

if (ZED_FOUND)
    mrover_add_nodelet(zed src/perception/zed_wrapper/*.c* src/perception/zed_wrapper src/perception/zed_wrapper/pch.hpp)
    mrover_nodelet_include_directories(zed ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    mrover_nodelet_link_libraries(zed ${ZED_LIBRARIES} ${SPECIAL_OS_LIBS} lie)
    mrover_nodelet_defines(zed
            ALLOW_BUILD_DEBUG # Ignore ZED warnings about Debug mode
            __CUDA_INCLUDE_COMPILER_INTERNAL_HEADERS__ # Eigen includes some files it should not, ignore
    )
endif ()

## Simulator

mrover_add_node(sim_arm_bridge src/simulator/arm_bridge/*.cpp)

mrover_add_gazebo_plugin(differential_drive_plugin_6w src/simulator/differential_drive_6w.cpp src)

mrover_add_gazebo_plugin(kinect_plugin src/simulator/gazebo_ros_openni_kinect.cpp src/simulator)
target_link_libraries(kinect_plugin PRIVATE gazebo_ros_camera_utils DepthCameraPlugin Eigen3::Eigen)
set_target_properties(kinect_plugin PROPERTIES CXX_CLANG_TIDY "")

### ======= ###
### Testing ###
### ======= ###

# Add C++ unit tests
catkin_add_gtest(example-cpp-test test/example/cpp_test.cpp)

# Python unit tests
catkin_add_nosetests(test/navigation/drive_test.py)
catkin_add_nosetests(test/teleop/teleop_test.py)
catkin_add_nosetests(test/util/SE3_test.py)
catkin_add_nosetests(test/util/SO3_test.py)

# Integration tests (python and c++)
find_package(rostest REQUIRED)
add_rostest(test/example/basic_integration_test.test)
add_rostest(test/integration/integration.test)
add_rostest(test/util/SE3_tf_test.test)

## Install

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
