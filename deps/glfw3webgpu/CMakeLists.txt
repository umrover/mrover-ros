cmake_minimum_required(VERSION 3.16)
project(glfw3webgpu LANGUAGES C)

# This is only meant to be included as a subdirectory in another project.
# It assumes that targets 'glfw' and 'webgpu' exist.
# Look at examples/CMakeLists.txt to see how to use it in a project.

# The glfw3webgpu target
add_library(glfw3webgpu STATIC src/glfw3webgpu.c)

# Find the required dependencies
find_package(glfw3 REQUIRED)

find_package(dawn QUIET)
if (dawn_FOUND)
    message(STATUS "Using Dawn system install")
else ()
    message(STATUS "Using Dawn from source")
    add_library(webgpu SHARED IMPORTED)

    set(WEBGPU_BUILD_DIR ${CMAKE_CURRENT_LIST_DIR}/../../deps/dawn/out/Release)
    if (APPLE)
        set(WEBGPU_SHARED_LIB ${WEBGPU_BUILD_DIR}/src/dawn/native/libwebgpu_dawn.dylib)
    else ()
        set(WEBGPU_SHARED_LIB ${WEBGPU_BUILD_DIR}/src/dawn/native/libwebgpu_dawn.so)
    endif ()
    if (EXISTS ${WEBGPU_SHARED_LIB})
        target_include_directories(webgpu INTERFACE ${CMAKE_CURRENT_LIST_DIR}/../../deps/dawn/include ${WEBGPU_BUILD_DIR}/gen/include)
        set_property(TARGET webgpu PROPERTY IMPORTED_LOCATION ${WEBGPU_SHARED_LIB})

        set(dawn_FOUND TRUE)
    else ()
        message(WARNING "Dawn not found. If on Ubuntu install with 'sudo apt install -f ./pkg/libdawn-dev.deb'. Or build from source with ./scripts/build_dawn.sh")
    endif ()
endif ()

# Set the install directory 
set(CMAKE_INSTALL_PREFIX "/usr/local")
message("Set the CMAKE_INSTALL_PREFIX to ${CMAKE_INSTALL_PREFIX}")

# Set the lib folder for when it needs to be linked against
set(${PROJECT_NAME}_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib")


# Set the include directory relative to install path
set(${PROJECT_NAME}_INSTALL_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")

# Include the directories necessary for building
target_include_directories	(${PROJECT_NAME} 
								PRIVATE
									headers
							)
# Set the public headers we would like to install
set(
    public_headers 
        headers/glfw3webgpu.h
)

# Install the binary
install (
			TARGETS ${PROJECT_NAME}
			DESTINATION ${${PROJECT_NAME}_LIBRARIES}
		)

# Install the public headers
install	(
			FILES
				${public_headers}
			DESTINATION ${${PROJECT_NAME}_INSTALL_INCLUDE_DIRS}
			COMPONENT all
		)

target_link_libraries(glfw3webgpu PUBLIC glfw webgpu)

if (APPLE)
  target_compile_options(glfw3webgpu PRIVATE -x objective-c)
  target_link_libraries(glfw3webgpu PRIVATE "-framework Cocoa" "-framework CoreVideo" "-framework IOKit" "-framework QuartzCore")
endif ()

set(CONFIG_FILES_DIR "${CMAKE_INSTALL_PREFIX}/share/cmake/${PROJECT_NAME}")

# NOT SURE what these do
install (
          	TARGETS ${PROJECT_NAME} 
          	EXPORT "${PROJECT_NAME}Targets"
			LIBRARY DESTINATION ${${PROJECT_NAME}_LIBRARIES}
			COMPONENT all
        )

install(EXPORT "${PROJECT_NAME}Targets"
    FILE "${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${namespace}
	DESTINATION ${CONFIG_FILES_DIR}
	COMPONENT all
)

include(CMakePackageConfigHelpers)

set(version 1.0)
# generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${version}"
    COMPATIBILITY AnyNewerVersion
)
# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION cmake
)
# install config files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
	DESTINATION ${CONFIG_FILES_DIR}
)

# where to find our CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)

