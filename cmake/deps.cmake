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
	find_package(glfw3webgpu REQUIRED)

	#add_subdirectory(deps/glfw3webgpu SYSTEM EXCLUDE_FROM_ALL)
    add_subdirectory(deps/imgui SYSTEM EXCLUDE_FROM_ALL)
    add_subdirectory(deps/webgpuhpp SYSTEM EXCLUDE_FROM_ALL)
endif ()

find_package(OpenCV REQUIRED)
find_package(ZED QUIET)
find_package(Eigen3 REQUIRED)

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

find_package(PkgConfig REQUIRED)
pkg_search_module(NetLink libnl-3.0 QUIET)
pkg_search_module(NetLinkRoute libnl-route-3.0 QUIET)
pkg_search_module(Gst gstreamer-1.0 QUIET)
pkg_search_module(GstApp gstreamer-app-1.0 QUIET)
