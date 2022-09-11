add_message_files(
        FILES
        # Add your new message here
)

# Starter Project
file(GLOB_RECURSE STARTER_PROJECT_PERCEPTION_SOURCES "src/*.cpp")
add_executable(starter_project_perception ${STARTER_PROJECT_PERCEPTION_SOURCES})
add_dependencies(starter_project_perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(starter_project_perception ${catkin_LIBRARIES} ${OpenCV_LIBS})

set(mrover_MESSAGE_FILES ${mrover_MESSAGE_FILES} PARENT_SCOPE)
