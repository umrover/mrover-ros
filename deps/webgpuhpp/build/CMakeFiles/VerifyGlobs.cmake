# CMAKE generated file: DO NOT EDIT!
# Generated by CMake Version 3.28
cmake_policy(SET CMP0009 NEW)

# IMGUI_SOURCES at imgui/CMakeLists.txt:6 (file)
file(GLOB_RECURSE NEW_GLOB LIST_DIRECTORIES false "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/*.cpp")
set(OLD_GLOB
  "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/backends/imgui_impl_glfw.cpp"
  "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/backends/imgui_impl_wgpu.cpp"
  "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/imgui.cpp"
  "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/imgui_draw.cpp"
  "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/imgui_tables.cpp"
  "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/imgui/imgui_widgets.cpp"
  )
if(NOT "${NEW_GLOB}" STREQUAL "${OLD_GLOB}")
  message("-- GLOB mismatch!")
  file(TOUCH_NOCREATE "/home/john/catkin_ws/src/mrover/deps/webgpuhpp/build/CMakeFiles/cmake.verify_globs")
endif()