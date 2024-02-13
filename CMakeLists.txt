cmake_minimum_required(VERSION 3.16)
project(mrover VERSION 2024.0.0 LANGUAGES CXX)

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
    find_package(TBB REQUIRED)
    link_libraries(TBB::tbb)
endif ()

include_directories(BEFORE SYSTEM src/preload)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json for clangd
if (NOT APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_link_options(-fuse-ld=lld) # LLVM lld is faster than GNU ld
endif ()
set(MROVER_CPP_COMPILE_OPTIONS -Wall -Wextra -pedantic)
if (MROVER_CI)
    list(APPEND MROVER_CPP_COMPILE_OPTIONS -Werror)
endif ()

if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/deps/dawn/out/Release)
    set(MROVER_BUILD_SIM ON)
else ()
    set(MROVER_BUILD_SIM OFF)
endif ()

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
        actionlib_msgs
)

extract_filenames(msg/*.msg MROVER_MESSAGE_FILES)

extract_filenames(srv/*.srv MROVER_SERVICE_FILES)

extract_filenames(action/*.action MROVER_ACTION_FILES)

set(MROVER_MESSAGE_DEPENDENCIES
        std_msgs
        sensor_msgs
        actionlib_msgs
)

set(MROVER_PARAMETERS
        config/DetectorParams.cfg
)

set(MROVER_CMAKE_INCLUDES
        starter_project/autonomy/AutonomyStarterProject.cmake
)

### ============ ###
### Dependencies ###
### ============ ###

if (MROVER_BUILD_SIM)
    # Apparently Assimp has different names on different systems
    # find_package is case-sensitive so try both
    find_package(Assimp QUIET)
    find_package(assimp QUIET)
    if (NOT Assimp_FOUND AND NOT assimp_FOUND)
        message(FATAL_ERROR "Assimp not found")
    endif ()

    find_package(Bullet REQUIRED)

    find_package(glfw3 REQUIRED)

    include(cmake/webgpu.cmake)

    add_subdirectory(deps/glfw3webgpu)
endif ()
find_package(OpenCV REQUIRED)
find_package(ZED QUIET)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(NetLink libnl-3.0)
pkg_search_module(NetLinkRoute libnl-route-3.0)
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

### ================================ ###
### Messages & Actions & Reconfigure ###
### ================================ ###

add_message_files(FILES ${MROVER_MESSAGE_FILES})

add_service_files(FILES ${MROVER_SERVICE_FILES})

add_action_files(DIRECTORY action FILES ${MROVER_ACTION_FILES})

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

## ESW

if (NOT APPLE)
    mrover_add_vendor_header_only_library(moteus src/esw/mjbots)
    mrover_add_header_only_library(can_device src/esw/can_device)
    mrover_add_library(motor_library src/esw/motor_library/*.cpp src/esw/motor_library)
    target_link_libraries(motor_library PUBLIC can_device moteus)

    if (NetLink_FOUND AND NetLinkRoute_FOUND)
        mrover_add_nodelet(can_driver src/esw/can_driver/*.cpp src/esw/can_driver src/esw/can_driver/pch.hpp)
        mrover_nodelet_link_libraries(can_driver nl-3 nl-route-3)
        # TODO(quintin): Fix this CMake hard coding
        #mrover_nodelet_link_libraries(can ${NetLink_LIBRARIES} ${NetLinkRoute_LIBRARIES}})
        mrover_nodelet_include_directories(can_driver /usr/include/libnl3)
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
#    mrover_add_esw_bridge_node(sa_bridge src/esw/sa_translator_bridge/*.cpp)
    mrover_add_esw_bridge_node(science_bridge src/esw/science_bridge/*.cpp)
    mrover_add_esw_bridge_node(brushless_test_bridge src/esw/brushless_test_bridge/*.cpp motor_library)
    mrover_add_esw_bridge_node(brushed_test_bridge src/esw/brushed_test_bridge/*.cpp)
    mrover_add_esw_bridge_node(test_arm_bridge src/esw/test_arm_bridge/*.cpp)
    #    mrover_add_esw_bridge_node(sa_sensor src/esw/fw/dirt_sensor/dirt_sensor_ros/dirt_sensor_ros.ino)
endif ()

## Perception

mrover_add_nodelet(tag_detector src/perception/tag_detector/*.cpp src/perception/tag_detector src/perception/tag_detector/pch.hpp)
mrover_nodelet_link_libraries(tag_detector opencv_core opencv_objdetect opencv_aruco opencv_imgproc opencv_highgui lie)

mrover_add_nodelet(click_ik src/perception/click_ik/*.cpp src/perception/click_ik src/perception/click_ik/pch.hpp)
mrover_nodelet_link_libraries(click_ik lie)

if (CUDA_FOUND)
    mrover_add_library(streaming src/esw/streaming/*.c* src/esw/streaming)
    #    target_link_libraries(streaming PUBLIC opencv_core opencv_cudacodec)
    target_link_libraries(streaming PUBLIC cuda nvidia-encode opencv_core)
    target_include_directories(streaming SYSTEM PUBLIC deps/nvenc)
    target_compile_definitions(streaming PUBLIC BOOST_ASIO_NO_DEPRECATED)
endif ()

if (ZED_FOUND)
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

## Simulator

if (MROVER_BUILD_SIM)
    mrover_add_nodelet(simulator src/simulator/*.cpp src/simulator src/simulator/pch.hpp)
    mrover_nodelet_include_directories(simulator ${ASSIMP_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} src/simulator/imgui)
    mrover_nodelet_link_libraries(simulator ${ASSIMP_LIBRARIES} ${BULLET_LIBRARIES} glfw3webgpu webgpu glfw opencv_core opencv_imgcodecs opencv_imgproc lie)
    mrover_nodelet_defines(simulator BOOST_THREAD_PROVIDES_FUTURE)
endif ()

# TODO

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
