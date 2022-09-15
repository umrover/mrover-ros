## Starter Project
# CMake is a build scripting system
# It is a program whose sole purpose is to generate Makefiles (or Ninja build files)
# It is used extensively in industry

add_message_files(
        FILES
        # Add your new message here
)

# Collect all cpp files in the src subdirectory to be used for perception
file(GLOB_RECURSE STARTER_PROJECT_PERCEPTION_SOURCES "src/*.cpp")
# Define a new CMake target, specifically a built C++ executable, that uses the found source files
add_executable(starter_project_perception ${STARTER_PROJECT_PERCEPTION_SOURCES})
# Ensure that our project builds after message generation
add_dependencies(starter_project_perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Link needed libraries
target_link_libraries(starter_project_perception ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Install our executable so that ROS knows about it
# This allows us to launch it with rosrun
install(TARGETS starter_project_perception
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Hack to make sub CMake directories generate messages
set(mrover_MESSAGE_FILES ${mrover_MESSAGE_FILES} PARENT_SCOPE)
