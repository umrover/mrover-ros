cmake_minimum_required(VERSION 3.16)
project(webgpu_hpp VERSION 6108 LANGUAGES CXX)

add_library(webgpu_hpp INTERFACE headers/webgpu.hpp)

# Set the install directory 
set(CMAKE_INSTALL_PREFIX "/usr/local")
message("Set the CMAKE_INSTALL_PREFIX to ${CMAKE_INSTALL_PREFIX}")

# Set the lib folder for when it needs to be linked against
set(${PROJECT_NAME}_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib")

# Set the include directory relative to install path
set(${PROJECT_NAME}_INSTALL_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")

# Get include directories from library and install them
get_target_property(${PROJECT_NAME}_INTERFACE_INCLUDE_DIRECTORIES webgpu_hpp SOURCES)
install (
			FILES
                ${${PROJECT_NAME}_INTERFACE_INCLUDE_DIRECTORIES}
            DESTINATION
				${${PROJECT_NAME}_INSTALL_INCLUDE_DIRS}
			COMPONENT all
        )

add_subdirectory(imgui/ SYSTEM EXCLUDE_FROM_ALL)

target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR})

set(CONFIG_FILES_DIR "${CMAKE_INSTALL_PREFIX}/share/cmake/${PROJECT_NAME}")

# Install files to allow cmake to find the library
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
