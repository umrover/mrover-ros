find_package(catkin REQUIRED)
catkin_package()

message("cmake path ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")
add_custom_target(mrover
	ALL
	COMMAND cargo build --manifest-path=${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
	#COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/cargo/release/my-project-binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/my-project-binary
	COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/target/debug/drive ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/drive
	COMMENT "Building my Rust library"
)