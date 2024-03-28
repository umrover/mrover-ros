cmake_minimum_required(VERSION 3.16)
project(glfw3webgpu LANGUAGES C)

# This is only meant to be included as a subdirectory in another project.
# It assumes that targets 'glfw' and 'webgpu' exist.
# Look at examples/CMakeLists.txt to see how to use it in a project.

# The glfw3webgpu target
add_library(glfw3webgpu STATIC glfw3webgpu.c)

find_package(glfw3 REQUIRED)
find_package(dawn REQUIRED)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")
message("Set the CMAKE_INSTALL_PREFIX to ${CMAKE_INSTALL_PREFIX}")

# Set the lib folder for when it needs to be linked against
set(${PROJECT_NAME}_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib")

# Set the include directory relative to install path
set(${PROJECT_NAME}_INSTALL_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")

target_include_directories  (
                              glfw3webgpu 
                                PUBLIC 
                                  .
                            )

set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${${PROJECT_NAME}_INSTALL_INCLUDE_DIRS})



install (
          TARGETS ${PROJECT_NAME} 
          EXPORT "${PROJECT_NAME}Targets"
          INCLUDES DESTINATION "${${PROJECT_NAME}_INSTALL_INCLUDE_DIRS}"
        )

target_link_libraries(glfw3webgpu PUBLIC glfw webgpu)

if (APPLE)
  target_compile_options(glfw3webgpu PRIVATE -x objective-c)
  target_link_libraries(glfw3webgpu PRIVATE "-framework Cocoa" "-framework CoreVideo" "-framework IOKit" "-framework QuartzCore")
endif ()

# generate and install export file
install(EXPORT "${PROJECT_NAME}Targets"
    FILE "${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${namespace}::
    DESTINATION cmake
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
    DESTINATION cmake
)
