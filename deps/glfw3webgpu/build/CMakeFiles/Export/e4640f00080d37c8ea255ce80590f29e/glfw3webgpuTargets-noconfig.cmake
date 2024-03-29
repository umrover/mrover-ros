#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "glfw3webgpu" for configuration ""
set_property(TARGET glfw3webgpu APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(glfw3webgpu PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libglfw3webgpu.a"
  )

list(APPEND _cmake_import_check_targets glfw3webgpu )
list(APPEND _cmake_import_check_files_for_glfw3webgpu "${_IMPORT_PREFIX}/lib/libglfw3webgpu.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
