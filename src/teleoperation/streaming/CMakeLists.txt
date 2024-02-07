cmake_minimum_required(VERSION 3.16)
project(mrover_streaming VERSION 2024.0.0 LANGUAGES CXX)

#set(CMAKE_TOOLCHAIN_FILE /home/quintin/Programming/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake

if (EMSCRIPTEN)
    message(STATUS "Building for the browser")
    if (CMAKE_BUILD_TYPE STREQUAL Release)
        message(STATUS "Adding extra optimizations")
        add_compile_options(-msimd128 -flto)
        add_link_options(-flto --bind)
    endif ()
    # See https://github.com/emscripten-core/emscripten/blob/main/src/settings.js
    add_link_options(-sINITIAL_MEMORY=512MB -sSTACK_SIZE=16MB)
endif ()

set(ENABLE_SDL OFF)
set(MROVER_DEPS_FOLDER ${CMAKE_CURRENT_LIST_DIR}/../../../deps)
add_subdirectory(${MROVER_DEPS_FOLDER}/libde265 ${CMAKE_BINARY_DIR}/libde265 EXCLUDE_FROM_ALL SYSTEM)

add_executable(stream_client stream_client.cpp)
target_link_libraries(stream_client PRIVATE de265 websocket.js)
target_include_directories(stream_client
        PRIVATE SYSTEM
        ${MROVER_DEPS_FOLDER}/libde265
        ${CMAKE_BINARY_DIR}/libde265
        /home/quintin/Programming/emsdk/upstream/emscripten/cache/sysroot/include
)
