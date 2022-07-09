find_package(catkin REQUIRED)
catkin_package()

add_custom_target(mrover
	ALL
	COMMAND cargo build 
	#COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/cargo/release/my-project-binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/my-project-binary
	COMMENT "Building my Rust library"
)