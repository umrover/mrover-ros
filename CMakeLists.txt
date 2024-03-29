cmake_minimum_required(VERSION 3.16)
project(mrover VERSION 2024.0.0 LANGUAGES CXX)

option(MROVER_IS_CI "Build for CI" OFF)
option(MROVER_RUN_CLANG_TIDY "Run clang-tidy" OFF)
option(MROVER_IS_JETSON "Build for the Jetson" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json for clangd

### ============= ###
### OS & Compiler ###
### ============= ###

if (APPLE)
    # Ensures that homebrew packages are never used over miniforge packages
    set(CMAKE_IGNORE_PATH /opt/homebrew)
    # Boost uses deprecated C++ library features
    # Thankfully libcxx has a compatibility mode for this
    add_definitions(
            -D_LIBCPP_ENABLE_CXX20_REMOVED_ALLOCATOR_MEMBERS
            -D_LIBCPP_ENABLE_CXX20_REMOVED_ALLOCATOR_VOID_SPECIALIZATION
    )

    find_package(fmt REQUIRED)
    link_libraries(fmt::fmt)
else ()
    # TODO(quintin): Fix this
    find_package(TBB QUIET)
    if (TBB_FOUND)
        link_libraries(TBB::tbb)
    endif ()
endif ()

if (NOT APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang") # Link with LLVM lld instead of GNU ld when using Clang on Linux, it is faster
    add_link_options(-fuse-ld=lld)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(MROVER_CPP_COMPILE_OPTIONS -Wall -Wextra -pedantic -Wno-missing-field-initializers)
    if (MROVER_IS_CI) # Make warnings errors in CI
        list(APPEND MROVER_CPP_COMPILE_OPTIONS -Werror)
    endif ()
endif ()

if (MROVER_IS_JETSON)
    add_definitions(-DMROVER_IS_JETSON)
endif ()

# Inject header files that get included before anything else, including system headers
include_directories(BEFORE SYSTEM src/preload)

include(cmake/macros.cmake)

# ROS packages list
set(MROVER_ROS_DEPENDENCIES
        urdf
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
)

extract_filenames(msg/*.msg MROVER_MESSAGE_FILES)

extract_filenames(srv/*.srv MROVER_SERVICE_FILES)

set(MROVER_MESSAGE_DEPENDENCIES
        std_msgs
        sensor_msgs
)

set(MROVER_PARAMETERS
        config/DetectorParams.cfg
        config/ObjectDetectorParams.cfg
)

set(MROVER_CMAKE_INCLUDES
        starter_project/autonomy/AutonomyStarterProject.cmake
)

### ============ ###
### Dependencies ###
### ============ ###

include(cmake/deps.cmake)

if (ZED_FOUND)
    # Anything newer than C++17 combined with libstdc++13 is not supported just yet by NVCC (the CUDA compiler)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
    set(CMAKE_CUDA_FLAGS "--diag-suppress 108,68")
    # Jetson Xavier NX/AGX has Volta 7.2 architecture
    # Perception Laptop (A4000, Quadro version of RTX 3080) has Ampere 8.6 architecture
    set(CMAKE_CUDA_ARCHITECTURES 72 86)
    enable_language(CUDA)
endif ()

find_package(catkin REQUIRED COMPONENTS ${MROVER_ROS_DEPENDENCIES})

catkin_python_setup()

foreach (MROVER_CMAKE_INCLUDE ${MROVER_CMAKE_INCLUDES})
    include(${MROVER_CMAKE_INCLUDE})
endforeach ()

set(BUILD_SHARED_LIBS ON)

### ======== ###
### Messages ###
### ======== ###

add_message_files(FILES ${MROVER_MESSAGE_FILES})

add_service_files(FILES ${MROVER_SERVICE_FILES})

generate_messages(DEPENDENCIES ${MROVER_MESSAGE_DEPENDENCIES})

generate_dynamic_reconfigure_options(${MROVER_PARAMETERS})

catkin_package()

### ======= ###
### Targets ###
### ======= ###

# Please browse the "Macros" section before adding anything here
# Lots of custom macros have been added to make adding new targets easier

## Libraries

mrover_add_library(lie src/util/lie/*.cpp src/util/lie)
target_link_libraries(lie PUBLIC MANIF::manif)

## ESW

if (NOT APPLE)
    mrover_add_vendor_header_only_library(moteus deps/mjbots)
    mrover_add_header_only_library(can_device src/esw/can_device)
    mrover_add_library(motor_library src/esw/motor_library/*.cpp src/esw/motor_library)
    target_link_libraries(motor_library PUBLIC can_device moteus)

    if (NetLink_FOUND AND NetLinkRoute_FOUND)
        mrover_add_nodelet(can_driver src/esw/can_driver/*.cpp src/esw/can_driver src/esw/can_driver/pch.hpp)
        mrover_nodelet_link_libraries(can_driver ${NetLink_LIBRARIES} ${NetLinkRoute_LIBRARIES})
        mrover_nodelet_include_directories(can_driver ${NetLink_INCLUDE_DIRS} ${NetLinkRoute_INCLUDE_DIRS})
    endif ()

    macro(mrover_add_esw_bridge_node name sources)
        mrover_add_node(${name} ${sources})
        target_link_libraries(${name} PRIVATE can_device motor_library)
    endmacro()

    mrover_add_esw_bridge_node(arm_hw_bridge src/esw/arm_hw_bridge/*.cpp)
    mrover_add_esw_bridge_node(arm_translator_bridge src/esw/arm_translator_bridge/*.cpp src/esw/arm_translator_bridge/*.hpp)
    mrover_add_esw_bridge_node(cache_bridge src/esw/cache_bridge/*.cpp)
    mrover_add_esw_bridge_node(drive_bridge src/esw/drive_bridge/*.cpp)
    mrover_add_esw_bridge_node(led_hw_bridge src/esw/led_hw_bridge/*.cpp)
    mrover_add_esw_bridge_node(led src/esw/led/*.cpp)
    mrover_add_esw_bridge_node(mast_gimbal_bridge src/esw/mast_gimbal_bridge/*.cpp)
    mrover_add_esw_bridge_node(pdb_bridge src/esw/pdb_bridge/*.cpp)
    mrover_add_esw_bridge_node(sa_hw_bridge src/esw/sa_hw_bridge/*.cpp)
    mrover_add_esw_bridge_node(sa_translator_bridge src/esw/sa_translator_bridge/*.cpp)
    mrover_add_esw_bridge_node(science_bridge src/esw/science_bridge/*.cpp)
    mrover_add_esw_bridge_node(brushless_test_bridge src/esw/brushless_test_bridge/*.cpp motor_library)
    mrover_add_esw_bridge_node(brushed_test_bridge src/esw/brushed_test_bridge/*.cpp)
    mrover_add_esw_bridge_node(test_arm_bridge src/esw/test_arm_bridge/*.cpp)
    mrover_add_esw_bridge_node(pdb_test_bridge src/esw/pdb_test_bridge/*.cpp)
    mrover_add_esw_bridge_node(arm_position_test_bridge src/esw/arm_position_test_bridge/*.cpp)
    #    mrover_add_esw_bridge_node(sa_sensor src/esw/fw/dirt_sensor/dirt_sensor_ros/dirt_sensor_ros.ino)
endif ()

## Perception

mrover_add_library(websocket_server src/esw/websocket_server/*.cpp src/esw/websocket_server)
target_compile_definitions(websocket_server PUBLIC BOOST_ASIO_NO_DEPRECATED)

mrover_add_nodelet(zed_tag_detector src/perception/tag_detector/zed/*.cpp src/perception/tag_detector/zed src/perception/tag_detector/zed/pch.hpp)
mrover_nodelet_link_libraries(zed_tag_detector opencv_core opencv_objdetect opencv_aruco opencv_imgproc lie)

mrover_add_nodelet(long_range_tag_detector src/perception/tag_detector/long_range_cam/*.cpp src/perception/tag_detector/long_range_cam src/perception/tag_detector/long_range_cam/pch.hpp)
mrover_nodelet_link_libraries(long_range_tag_detector opencv_core opencv_objdetect opencv_aruco opencv_imgproc lie)

mrover_add_nodelet(usb_camera src/perception/usb_camera/*.cpp src/perception/usb_camera src/perception/usb_camera/pch.hpp)
mrover_nodelet_link_libraries(usb_camera opencv_core opencv_objdetect opencv_aruco opencv_imgproc opencv_highgui)

if (CUDA_FOUND)
    #    mrover_add_node(nv_vid_codec_h265_enc src/esw/nv_vid_codec_h265_enc/*.c*)
    #    target_link_libraries(nv_vid_codec_h265_enc PUBLIC cuda nvidia-encode opencv_core opencv_imgproc streaming)
    #    target_include_directories(nv_vid_codec_h265_enc SYSTEM PUBLIC deps/nvenc)
endif ()

mrover_add_nodelet(gst_websocket_streamer src/esw/gst_websocket_streamer/*.c* src/esw/gst_websocket_streamer src/esw/gst_websocket_streamer/pch.hpp)
mrover_nodelet_link_libraries(gst_websocket_streamer PRIVATE websocket_server ${Gst_LIBRARIES} ${GstApp_LIBRARIES})
mrover_nodelet_include_directories(gst_websocket_streamer ${Gst_INCLUDE_DIRS} ${GstApp_INCLUDE_DIRS})

if (ZED_FOUND)
    mrover_add_nodelet(object_detector src/perception/object_detector/*.c* src/perception/object_detector src/perception/object_detector/pch.hpp)
    mrover_nodelet_link_libraries(object_detector PRIVATE opencv_core opencv_dnn opencv_imgproc lie nvinfer nvonnxparser tbb)
    mrover_nodelet_defines(object_detector
            __CUDA_INCLUDE_COMPILER_INTERNAL_HEADERS__ # Eigen includes some files it should not, ignore
    )
    # TODO(john): Update object detector to use new API
    target_compile_options(object_detector_nodelet PRIVATE -Wno-deprecated-declarations)
    # Temporary
    mrover_nodelet_link_libraries(object_detector PRIVATE opencv_imgcodecs opencv_highgui)

    mrover_add_nodelet(zed src/perception/zed_wrapper/*.c* src/perception/zed_wrapper src/perception/zed_wrapper/pch.hpp)
    mrover_nodelet_include_directories(zed ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    mrover_nodelet_link_libraries(zed ${ZED_LIBRARIES} ${SPECIAL_OS_LIBS} lie)
    mrover_nodelet_defines(zed
            ALLOW_BUILD_DEBUG # Ignore ZED warnings about Debug mode
            __CUDA_INCLUDE_COMPILER_INTERNAL_HEADERS__ # Eigen includes some files it should not, ignore
    )
endif ()

## Teleoperation

mrover_add_node(arm_controller src/teleoperation/arm_controller/*.cpp)
target_link_libraries(arm_controller PRIVATE lie)

## Simulator

if (MROVER_BUILD_SIM)
    mrover_add_nodelet(simulator src/simulator/*.cpp src/simulator src/simulator/pch.hpp)
    mrover_nodelet_include_directories(simulator ${ASSIMP_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
    mrover_nodelet_link_libraries(simulator ${ASSIMP_LIBRARIES} ${BULLET_LIBRARIES} glfw3webgpu webgpu glfw opencv_core opencv_imgcodecs opencv_imgproc webgpu_hpp imgui lie)
    mrover_nodelet_defines(simulator BOOST_THREAD_PROVIDES_FUTURE)
endif ()

### ======= ###
### Testing ###
### ======= ###

# Add C++ unit tests
catkin_add_gtest(example-cpp-test test/example/cpp_test.cpp)

# Python unit tests
catkin_add_nosetests(test/navigation/drive_test.py)
catkin_add_nosetests(test/util/SE3_test.py)
catkin_add_nosetests(test/util/SO3_test.py)

# Integration tests (python and c++)
# Note(quintin): Commented these out because they ignore the Python virtual environment
#                Most of our code relies on newer Python features so they fail to initialize
#find_package(rostest REQUIRED)
#add_rostest(test/example/basic_integration_test.test)
#add_rostest(test/integration/integration.test)
#add_rostest(test/util/SE3_tf_test.test)

## Install

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
