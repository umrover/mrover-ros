# The simulator will only be built if Dawn (low-level graphics API) is found:
# 1) Installed system-wide with the .deb package in the pkg/ folder. This is ONLY for Ubuntu 20
#    If this is the case then "find_package" will set "dawn_FOUND" to true
# 2) Built from source with the build_dawn.sh script. This is for all other systems (non-Ubuntu, macOS, etc.)
#    If this is the case libwebgpu_dawn.* will be found in deps/dawn/out/Release and "dawn_FOUND" will be set to true

find_package(dawn QUIET)
if (dawn_FOUND)
    message(STATUS "Using Dawn system install")
else ()
    message(STATUS "Using Dawn from source")
    add_library(webgpu SHARED IMPORTED)

    set(WEBGPU_BUILD_DIR ${CMAKE_CURRENT_LIST_DIR}/../deps/dawn/out/Release)
    if (APPLE)
        set(WEBGPU_SHARED_LIB ${WEBGPU_BUILD_DIR}/src/dawn/native/libwebgpu_dawn.dylib)
    else ()
        set(WEBGPU_SHARED_LIB ${WEBGPU_BUILD_DIR}/src/dawn/native/libwebgpu_dawn.so)
    endif ()
    if (EXISTS ${WEBGPU_SHARED_LIB})
        target_include_directories(webgpu INTERFACE ${CMAKE_CURRENT_LIST_DIR}/../deps/dawn/include ${WEBGPU_BUILD_DIR}/gen/include)
        set_property(TARGET webgpu PROPERTY IMPORTED_LOCATION ${WEBGPU_SHARED_LIB})

        set(dawn_FOUND TRUE)
    else ()
        message(WARNING "Dawn not found. If on Ubuntu install with 'sudo apt install -f ./pkg/libdawn-dev.deb'. Or build from source with ./scripts/build_dawn.sh")
    endif ()
endif ()

option(MROVER_BUILD_SIM "Build the simulator" ${dawn_FOUND})

if (MROVER_BUILD_SIM)
    # Apparently Assimp has different names on different systems
    # find_package is case-sensitive so try both
    find_package(Assimp NAMES Assimp assimp QUIET)
    if (NOT Assimp_FOUND)
        message(FATAL_ERROR "Assimp not found")
    endif ()

    find_package(Bullet REQUIRED)
    find_package(glfw3 REQUIRED)

    #Find glfw3webgpu or build from source
    find_package(glfw3webgpu REQUIRED)
    if(NOT glfw3webgpu_FOUND)
        add_subdirectory(deps/glfw3webgpu SYSTEM EXCLUDE_FROM_ALL)
    endif()

    #Find webgpu_hpp or build from source
	find_package(webgpu_hpp REQUIRED)
    if(NOT webgpu_hpp_FOUND)
        add_subdirectory(deps/webgpuhpp SYSTEM EXCLUDE_FROM_ALL)
    endif()

    #Find imgui or build from source
	find_package(imgui REQUIRED)
    if(NOT imgui_FOUND)
        add_subdirectory(deps/imgui SYSTEM EXCLUDE_FROM_ALL)
    endif()

endif ()

find_package(OpenCV REQUIRED)
find_package(ZED QUIET)
find_package(Eigen3 REQUIRED)

# Same idea as dawn, ideally installed via a package, but if not then build from source
find_package(manif QUIET)
if (NOT manif_FOUND)
    if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/../deps/manif/include/manif)
        add_subdirectory(deps/manif SYSTEM)
        add_library(MANIF::manif ALIAS manif)

        set(manif_FOUND TRUE)
    else ()
        message(FATAL_ERROR "Manif not found. If on Ubuntu install with 'sudo apt install -f ./pkg/libmanif-dev.deb'. Or build from source with 'submodule update --init deps/manif' and make sure it is non-empty")
    endif ()
endif ()

# These are old packages so they do not support "find_package" and must be found with pkg-config
# Thankfully CMake has a built-in module for this
find_package(PkgConfig REQUIRED)
pkg_search_module(NetLink libnl-3.0 QUIET)
pkg_search_module(NetLinkRoute libnl-route-3.0 QUIET)
pkg_search_module(Gst gstreamer-1.0 QUIET)
pkg_search_module(GstApp gstreamer-app-1.0 QUIET)
