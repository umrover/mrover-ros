cmake_minimum_required(VERSION 3.16)
project(imgui VERSION 1.90 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find webgpu and glfw
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

# Grab all of the sources for imgui
file(GLOB_RECURSE IMGUI_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

# Create the imgui target and include the headers
add_library(imgui ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Grab all of the files that need to be included for imgui and store them in im
get_target_property(${PROJECT_NAME}_INTERFACE_INCLUDE_DIRECTORIES imgui INTERFACE_INCLUDE_DIRECTORIES)
file(GLOB_RECURSE ${PROJECT_NAME}_INTERFACE_HEADERS ${${PROJECT_NAME}_INTERFACE_INCLUDE_DIRECTORIES}/*.h*)

# Install the library binary
install	(
			TARGETS
				${PROJECT_NAME}
			DESTINATION
				${PROJECT_NAME}_LIBRARIES
		)

# Install the header files for imgui
install	(
			FILES
				${PROJECT_NAME}_INTERFACE_HEADERS
			DESTINATION
				${PROJECT_NAME}_INSTALL_INCLUDE_DIRS
		)

message("INTERFACE_INCLUDE_DIRECTORIES ${imgui_INTERFACE_INCLUDE_DIRECTORIES}")

message("INTERFACE_INCLUDE_DIRECTORIES ${${PROJECT_NAME}_INTERFACE_HEADERS}")

target_link_libraries(imgui PUBLIC glfw webgpu)
