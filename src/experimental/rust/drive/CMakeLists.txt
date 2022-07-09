find_package(catkin REQUIRED)
catkin_package()

message("cmake path ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")
add_custom_target(mrover
	ALL
	COMMAND cargo build --manifest-path=${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
	COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/target/debug/drive ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/drive
	COMMENT "Building my Rust Node"
)