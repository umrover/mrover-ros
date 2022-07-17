find_package(catkin REQUIRED)
catkin_package()

message("Building demo rust project")
add_custom_target(demo
	ALL
	COMMAND cargo build --manifest-path=${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
	#COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/target/debug/demo ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/demo
	COMMENT "Building my Rust Node"
)