cmake_minimum_required(VERSION 3.16)
project(mrover VERSION 2024.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_link_options(-fuse-ld=lld-16)
endif ()
set(MROVER_CPP_COMPILE_OPTIONS -Wall -Wextra -Werror -pedantic)

# Generate compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS packages list
set(MROVER_PACKAGES
        rospy
        roscpp
        std_msgs
        message_generation
        dynamic_reconfigure
        rostest
        sensor_msgs
)

# Message files
set(MROVER_MESSAGE_FILES
        AvailableCameras.msg
        Calibrated.msg
        CalibrationStatus.msg
        CameraCmd.msg
        Carousel.msg
        Chassis.msg
        Course.msg
        Diagnostic.msg
        EnableAuton.msg
        GPSWaypoint.msg
        GPSPointList.msg
        HeaterData.msg
        ImuAndMag.msg
        LimitSwitchData.msg
        MastGimbal.msg
        JointLock.msg
        MoteusState.msg
        MotorsStatus.msg
        NavMetadata.msg
        NetworkBandwidth.msg
        ScienceTemperature.msg
        Spectral.msg
        Waypoint.msg
        WaypointType.msg
        GPSPointList.msg
)

# Service files
set(MROVER_SERVICE_FILES
        AdjustMotors.srv
        CalibrateMotors.srv
        ChangeArmMode.srv
        ChangeCameras.srv
        ChangeHeaterState.srv
        ChangeServoAngle.srv
        EnableDevice.srv
        FetchMessageFromPackage.srv
        FetchMessageFromPackage.srv
        FetchPackages.srv
        PublishEnableAuton.srv
        PublishCourse.srv
        ResetCameras.srv
)

# Generate messages list
set(MROVER_ROS_MESSAGES
        sensor_msgs
        std_msgs
        sensor_msgs
)

# Dynamic reconfigure parameter file list
set(MROVER_PARAMETERS
        config/DetectorParams.cfg
)

# catkin packages list
set(MROVER_CATKIN_PACKAGES
        roscpp rospy std_msgs message_runtime
)

macro(rosify_cpp_target_macro target)
    target_link_libraries(${target} PRIVATE ${catkin_LIBRARIES})
    target_include_directories(${target} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} src/util)
    add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${MROVER_CPP_COMPILE_OPTIONS}>)

    install(TARGETS ${target}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endmacro()

macro(add_cpp_library_macro name sources includes)
    file(GLOB_RECURSE CPP_LIB_SOURCES ${sources})
    add_library(${name} ${CPP_LIB_SOURCES})
    target_include_directories(${name} PUBLIC ${includes})
    rosify_cpp_target_macro(${name})
endmacro()

macro(add_cpp_node_macro name sources)
    file(GLOB_RECURSE CPP_NODE_SOURCES ${sources})
    add_executable(${name} ${CPP_NODE_SOURCES})
    rosify_cpp_target_macro(${name})
endmacro()

macro(add_cpp_nodelet_macro name sources includes)
    # A nodelet runs inside another process so it is a library
    add_cpp_library_macro(${name} ${sources} ${includes})
endmacro()

macro(add_gazebo_plugin_macro name sources includes)
    add_cpp_library_macro(${name} ${sources} ${includes})

    # TODO: find a proper variable name that points to /opt/ros/noetic/lib
    target_link_directories(${name} PRIVATE ${GAZEBO_LIBRARY_DIRS} /opt/ros/noetic/lib)
    target_link_libraries(${name} PRIVATE ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
    target_include_directories(${name} SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
    set_target_properties(${name} PROPERTIES CXX_STANDARD 17)
endmacro()

# launch install macro
macro(install_launch_macro)
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
endmacro()

macro(add_tests_macro)
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
endmacro()

# Subdirectories before message declarations
set(MROVER_CMAKE_INCLUDES "")


# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Specify build details by appending to lists
# and implementing the some extra macros
# Add new devices as elseif blocks
# Select device with --cmake-flags -D DEVICE=<fill in device>
# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

if (${DEVICE} MATCHES "raspi4")
    # -=-=-=-=-
    # Lists
    # -=-=-=-=-

    # Add any raspi4 specific packages here
    # list(APPEND _PACKAGES )

    # -=-=-=-=-
    # Macros
    # -=-=-=-=-

    macro(include_directories_macro)
        include_directories(
                ${catkin_INCLUDE_DIRS}
        )
    endmacro()

    # define an add and link macro
    # Put items here to build
    macro(add_and_link_macro)
        add_cpp_node_macro(brushed_motors "src/esw/brushed_motors/*")
    endmacro()
else ()
    # -=-=-=-=-
    # Lists
    # -=-=-=-=-

    # Add any laptop specific packages here
    list(APPEND MROVER_PACKAGES
            tf2_geometry_msgs
            tf2_ros
            tf2
            gazebo_ros
    )

    # append subdirectories
    list(APPEND MROVER_CMAKE_INCLUDES
            starter_project/autonomy/AutonomyStarterProject.cmake
    )


    # -=-=-=-=-
    # Macros
    # -=-=-=-=-

    # These packages need to be found individually
    macro(add_packages_macro)
        find_package(OpenCV REQUIRED)
        find_package(gazebo REQUIRED)
        find_package(Eigen3 REQUIRED)
        find_package(ZED 2 QUIET)
        if (ZED_FOUND)
            # Anything newer than C++17 combined with libstdc++13 is not supported just yet by NVCC (the CUDA compiler)
            set(CMAKE_CUDA_STANDARD 17)
            set(CMAKE_CUDA_STANDARD_REQUIRED ON)
            set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
            # Jetson Xavier NX is Volta 7.2 architecture
            # Perception Laptop (A4000, Quadro version of RTX 3080) is Ampere 8.6 architecture
            set(CMAKE_CUDA_ARCHITECTURES 72 86)
            enable_language(CUDA)
        endif ()
    endmacro()

    # define an add and link macro
    # Put items here to build
    macro(add_and_link_macro)
        add_cpp_nodelet_macro(tag_detector_nodelet src/perception/tag_detector/*.cpp src/perception/tag_detector)
        target_precompile_headers(tag_detector_nodelet PRIVATE src/perception/tag_detector/pch.hpp)
        target_link_libraries(tag_detector_nodelet PRIVATE opencv_core opencv_objdetect opencv_aruco opencv_imgproc lie)

        add_cpp_node_macro(brushed_motors "src/esw/brushed_motors/*.cpp")

        add_cpp_library_macro(lie "src/util/lie/*.cpp" "src/util/lie")

        add_gazebo_plugin_macro(differential_drive_plugin_6w "src/gazebo/differential_drive_6w.cpp" "src")

        add_gazebo_plugin_macro(kinect_plugin "src/gazebo/gazebo_ros_openni_kinect.cpp" "src/gazebo")
        target_link_libraries(kinect_plugin PRIVATE gazebo_ros_camera_utils DepthCameraPlugin Eigen3::Eigen)
        set_target_properties(kinect_plugin PROPERTIES CXX_CLANG_TIDY "")

        if (ZED_FOUND)
            #            add_cpp_node_macro(zed_node "src/perception/zed_wrapper/*.c*")
            #            target_link_options(zed_node PRIVATE "LINKER:--copy-dt-needed-entries")
            #            target_include_directories(zed_node SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
            #            target_link_libraries(zed_node PRIVATE ${ZED_LIBRARIES} ${OpenCV_LIBRARIES} ${SPECIAL_OS_LIBS} lie tag_detector_nodelet)

            add_cpp_nodelet_macro(zed_nodelet "src/perception/zed_wrapper/*.c*" , "src/perception/zed_wrapper")
            target_precompile_headers(zed_nodelet PRIVATE src/perception/zed_wrapper/pch.hpp)
            target_include_directories(zed_nodelet SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
            target_link_libraries(zed_nodelet PRIVATE ${ZED_LIBRARIES} ${OpenCV_LIBRARIES} ${SPECIAL_OS_LIBS} lie)
            target_compile_definitions(zed_nodelet PRIVATE
                    MROVER_IS_NODELET # Generate code relevant to nodelet
                    ALLOW_BUILD_DEBUG # Ignore ZED warnings about Debug mode
                    __CUDA_INCLUDE_COMPILER_INTERNAL_HEADERS__ # Eigen includes some files it should not, ignore
            )
        endif ()
    endmacro()
endif ()


# 3. Find Packages
find_package(
        catkin REQUIRED COMPONENTS
        ${MROVER_PACKAGES}
)

if (COMMAND add_packages_macro)
    add_packages_macro()
endif ()


# 4. Python module support
catkin_python_setup()


# 4.5. CMake includes before message declarations
foreach (CMAKE_INCLUDE ${MROVER_CMAKE_INCLUDES})
    include(${CMAKE_INCLUDE})
endforeach ()


# 5. Message Generators (add_xxx)
add_message_files(
        FILES
        ${MROVER_MESSAGE_FILES}
)

add_service_files(
        FILES
        ${MROVER_SERVICE_FILES}
)


# 6. Invoke messages (generate_messages)
generate_messages(
        DEPENDENCIES
        ${MROVER_ROS_MESSAGES}
)

generate_dynamic_reconfigure_options(
        ${MROVER_PARAMETERS}
)


# 7. Specify package build info export (catkin_package)
catkin_package(
        CATKIN_DEPENDS
        ${MROVER_CATKIN_PACKAGES}
)


if (COMMAND add_and_link_macro)
    add_and_link_macro()
endif ()


# 9. Tests to build
if (COMMAND add_tests_macro)
    add_tests_macro()
endif ()


# 10. Install rules
install_launch_macro()
if (COMMAND additional_install_macro)
    additional_install_macro()
endif ()
