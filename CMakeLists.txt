cmake_minimum_required(VERSION 3.16)
project(mrover)

## Compile as C++17, supported in ROS Kinetic and newer, enable some static analysis
set(MROVER_CPP_COMPILE_OPTIONS -std=c++17 -Wall -Wextra -Werror -pedantic)

# clang-tidy
if (CMAKE_BUILD_TYPE EQUAL "RELEASE")
    set(CMAKE_CXX_CLANG_TIDY clang-tidy-12)
endif ()

# find_package
set(MROVER_PACKAGES
        rospy
        std_msgs
        message_generation
        dynamic_reconfigure
        rostest
        )

# link_directories
set(MROVER_LINK_DIRS "")

# message files
set(MROVER_MESSAGE_FILES
        CalibrationStatus.msg
        Chassis.msg
        Course.msg
        EnableAuton.msg
        GimbalCmd.msg
        GPSWaypoint.msg
        ODriveState.msg
        Waypoint.msg
        WaypointType.msg
        WheelData.msg
        )

# service files
set(MROVER_SERVICE_FILES
        PublishCourse.srv
        )

# generate messages list
set(MROVER_ROS_MESSAGES
        std_msgs
        )

# dynamic reconfigure
set(MROVER_PARAMETERS
        config/DetectorParams.cfg
        )

# catkin_package
set(MROVER_CATKIN_PACKAGES
        roscpp rospy std_msgs message_runtime
        )

macro(add_cpp_node_macro name sources)
    file(GLOB_RECURSE CPP_NODE_SOURCES ${sources})
    add_executable(${name} ${CPP_NODE_SOURCES})
    add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${name} ${catkin_LIBRARIES})
    target_compile_options(${name} PRIVATE ${MROVER_CPP_COMPILE_OPTIONS})

    install(TARGETS ${name}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
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
    catkin_add_nosetests(test/navigation/drive.py)
    catkin_add_nosetests(test/util/SE3_test.py)
    catkin_add_nosetests(test/util/SO3_test.py)

    # Integration tests (python and c++)
    find_package(rostest REQUIRED)
    add_rostest(test/example/basic_integration_test.test)
    add_rostest(test/integration/integration.test)
    add_rostest(test/util/SE3_tf_test.test)
endmacro()

# Subdirectories before message declarations
set(CMAKE_SUBDIRS "")


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

else ()
    # -=-=-=-=-
    # Lists
    # -=-=-=-=-

    # Add any laptop specific packages here
    list(APPEND MROVER_PACKAGES
            roscpp
            tf2_geometry_msgs
            tf2_ros
            tf2
            visualization_msgs
            image_transport
            cv_bridge
            sensor_msgs
            rosbridge_server
            teleop_twist_joy
            gazebo_ros
            moveit_commander
            moveit_core
            moveit_fake_controller_manager
            moveit_kinematics
            moveit_msgs
            moveit_planners_chomp
            moveit_planners_ompl
            moveit_ros_benchmarks
            moveit_ros_control_interface
            moveit_ros_manipulation
            moveit_ros_move_group
            moveit_ros_occupancy_map_monitor
            moveit_ros_perception
            moveit_ros_planning
            moveit_ros_planning_interface
            moveit_ros_robot_interaction
            moveit_ros_visualization
            moveit_ros_warehouse
            moveit_setup_assistant
            moveit_simple_controller_manager
            )

    # append link directories
    list(APPEND MROVER_LINK_DIRS
            ${GAZEBO_LIBRARY_DIRS}
            )

    # append subdirectories
    list(APPEND CMAKE_SUBDIRS
            starter_project/teleop
            )

    # -=-=-=-=-
    # Macros
    # -=-=-=-=-

    # These packages need to be found individually
    macro(add_packages_macro)
        find_package(OpenCV REQUIRED)
        find_package(gazebo REQUIRED)
        find_package(PCL REQUIRED)
        find_package(Eigen3 REQUIRED)
    endmacro()

    macro(include_directories_macro)
        include_directories(
                ${catkin_INCLUDE_DIRS}
                ${GAZEBO_INCLUDE_DIRS}
                ${OpenCV_INCLUDE_DIRS}
        )
    endmacro()

    # define an add and link macro
    # Put items here to build
    macro(add_and_link_macro)
        add_cpp_node_macro(aruco_detect "src/perception/*.cpp")
        target_link_libraries(aruco_detect ${OpenCV_LIBS} ${PCL_LIBRARIES})

        add_library(differential_drive_plugin_6w src/gazebo/differential_drive_6w.cpp)
        add_dependencies(differential_drive_plugin_6w ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
        target_link_libraries(differential_drive_plugin_6w ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
        target_compile_options(differential_drive_plugin_6w PRIVATE ${MROVER_CPP_COMPILE_OPTIONS})

        add_library(kinect_plugin src/gazebo/gazebo_ros_openni_kinect.cpp)
        add_dependencies(kinect_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
        target_link_libraries(kinect_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} gazebo_ros_camera_utils DepthCameraPlugin ${catkin_LIBRARIES} Eigen3::Eigen)
        set_target_properties(kinect_plugin PROPERTIES CXX_CLANG_TIDY "")
    endmacro()

    macro(additional_install_macro)
        install(TARGETS differential_drive_plugin_6w
                ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
                )

        install(TARGETS kinect_plugin
                ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
                )
    endmacro()

    # Set policy
    if (POLICY CMP0054)
        cmake_policy(SET CMP0054 NEW)
    endif ()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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


# 4.5. Subdirectories before message declarations
if (NOT "${CMAKE_SUBDIRS}" MATCHES "")
    add_subdirectory(
            ${CMAKE_SUBDIRS}
    )
endif ()


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


# 8. Libraries/Executables to build (add_executable)
# These only exist after find_packages
link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        /opt/ros/noetic/lib/
)

if (COMMAND include_directories_macro)
    include_directories_macro()
endif ()

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